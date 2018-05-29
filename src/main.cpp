//
//  main.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "StandardIncludes.h"

#include "ImageReader.h"
#include "Tracker.h"
#include "RapidExposureTimeEstimator.h"
#include "Database.h"
#include "NonlinearOptimizer.h"
#include "CLI11.hpp"

using namespace std;

// Optimization thread handle
pthread_t opt_thread = 0;

// This variable indicates if currently an optimization task is running in a second thread
pthread_mutex_t g_is_optimizing_mutex;
bool g_is_optimizing = false;

struct Settings{
    int start_image_index;      // Start image index.
    int end_image_index;        // End image index.
    int image_width;            // Image width to resize to.
    int image_height;           // Image height to resize to.
    int visualize_cnt;          // Visualize every visualize_cnt image (tracking + correction), rather slow.
    int tracker_patch_size;     // Image patch size used in tracker.
    int nr_pyramid_levels;      // Number of image pyramid levels used in tracker.
    int nr_active_features;     // Number of features maintained for each frame.
    int nr_images_rapid_exp;    // Number of images for rapid exposure time estimation.
    int nr_active_frames;       // Number of frames maintained in database.
    int keyframe_spacing;       // Spacing for sampling keyframes in backend optimization.
    int min_keyframes_valid;    // Minimum amount of keyframes a feature should be present to be included in optimization.
    string image_folder;        // Image folder.
    string exposure_gt_file;    // Exposure times ground truth file.
    string calibration_mode;    // Choose "online" or "batch".
};

void run_batch_optimization_task(NonlinearOptimizer *optimizer)
{
    std::cout << "START BATCH OPTIMIZATION" << std::endl;
    
    optimizer->fetchResponseVignetteFromDatabase();

    // Perform optimization (since its offline just do )
    optimizer->evfOptimization(false);
    std::cout << "EVF 1 DONE" << std::endl;
    optimizer->radianceFullOptimization();
    std::cout << "RAD 1 DONE" << std::endl;
    optimizer->evfOptimization(false);
    std::cout << "EVF 2 DONE" << std::endl;
    optimizer->radianceFullOptimization();
    std::cout << "RAD 2 DONE" << std::endl;
    optimizer->evfOptimization(false);
    std::cout << "EVF 3 DONE" << std::endl;
    
    // Smooth optimization data
    optimizer->smoothResponse();
    
    // Initialize the inverse response vector with the current inverse response estimate
    // (in order to write it to the database later + visualization)
    // better to do this here since currently the inversion is done rather inefficiently and not to slow down tracking
    optimizer->getInverseResponseRaw(optimizer->m_raw_inverse_response);
    std::cout << "END OPTIMIZATION" << std::endl;
}

void *run_optimization_task(void* thread_arg)
{
    std::cout << "START OPTIMIZATION" << std::endl;
    
    pthread_mutex_lock(&g_is_optimizing_mutex);
    g_is_optimizing = true;
    pthread_mutex_unlock(&g_is_optimizing_mutex);
    
    // The nonlinear optimizer contains all the optimization information
    NonlinearOptimizer* optimizer = (NonlinearOptimizer*)thread_arg;
    
    optimizer->fetchResponseVignetteFromDatabase();

    // Perform optimization
    optimizer->evfOptimization(false);
    optimizer->evfOptimization(false);
    optimizer->evfOptimization(false);
    
    // Smooth optimization data
    optimizer->smoothResponse();
    
    // Initialize the inverse response vector with the current inverse response estimate
    // (in order to write it to the database later + visualization)
    // better to do this here since currently the inversion is done rather inefficiently and not to slow down tracking
    optimizer->getInverseResponseRaw(optimizer->m_raw_inverse_response);
    
    pthread_mutex_lock(&g_is_optimizing_mutex);
    g_is_optimizing = false;
    pthread_mutex_unlock(&g_is_optimizing_mutex);
    std::cout << "END OPTIMIZATION" << std::endl;
    
    pthread_exit(NULL);
}

// Split a string into substrings given a char delimiter
std::vector<string> split(const string &s, char delim) {
    stringstream ss(s);
    string item;
    vector<string> tokens;
    while (getline(ss, item, delim)) {
        tokens.push_back(item);
    }
    return tokens;
}

int run_batch_calibration(Settings *run_settings,std::vector<double> gt_exp_times)
{
    int safe_zone_size = 0;

    double vis_exponent = 1.0;

    //  Set up the object to read new images from
    ImageReader image_reader(run_settings->image_folder, cv::Size(run_settings->image_width, run_settings->image_height));

    // Set up the information database
    Database database(run_settings->image_width,run_settings->image_height);

    // Setup the rapid  exposure time estimator
    RapidExposureTimeEstimator exposure_estimator(run_settings->nr_images_rapid_exp, &database);

    // Setup the nonlinear optimizer
    NonlinearOptimizer backend_optimizer(run_settings->keyframe_spacing,
                                         &database,
                                         safe_zone_size,
                                         run_settings->min_keyframes_valid,
                                         run_settings->tracker_patch_size);

    // Set up the object that handles the tracking and receives new images, extracts features
    Tracker tracker(run_settings->tracker_patch_size,run_settings->nr_active_features,run_settings->nr_pyramid_levels,&database);
    
    int num_images = image_reader.getNumImages();
    
    // Run over all input images, track the new image, estimate exposure time and optimize other parameters in the background
    for(int i = run_settings->start_image_index; i < num_images && (run_settings->end_image_index < 0 || i < run_settings->end_image_index); i++)
    {
        std::cout << "image: " << i << std::endl;

        // Read GT exposure time if available for this frame
        // Todo: Check if index is out of bounds, if gt exp file has not enough lines
        double gt_exp_time = -1.0;
        if(gt_exp_times.size() > 0)
        {
            gt_exp_time = gt_exp_times.at(i);
        }
        
        // Read next input image
        cv::Mat new_image = image_reader.readImage(i);
        
        // Track input image (+ time the result)
        tracker.trackNewFrame(new_image,gt_exp_time);

        // Rapid exposure time estimation (+ time the result)
        double exposure_time = exposure_estimator.estimateExposureTime();
        database.m_tracked_frames.at(database.m_tracked_frames.size()-1).m_exp_time = exposure_time;
        database.visualizeRapidExposureTimeEstimates(vis_exponent);

        // Remove the exposure time from the radiance estimates
        std::vector<Feature*>* features = &database.m_tracked_frames.at(database.m_tracked_frames.size()-1).m_features;
        for(int k = 0;k < features->size();k++)
        {
            for(int r = 0;r < features->at(k)->m_radiance_estimates.size();r++)
            {
                features->at(k)->m_radiance_estimates.at(r) /= exposure_time;
            }
        }
  
        // Visualize tracking
        if(i%run_settings->visualize_cnt == 0)
            database.visualizeTracking();

        //check if number of frames in database are enough to start the optimization
        if(database.m_tracked_frames.size() != run_settings->nr_active_frames)
        {
            continue;
        }
        
        // Required number of frames has been reached, calibrate all of them
        
        // Try to fetch a new optimization block
        bool succeeded = backend_optimizer.extractOptimizationBlock();
        
        if(succeeded)
        {
            // Optimize all frames
            run_batch_optimization_task(&backend_optimizer);

            // Show optimization result
            vis_exponent = backend_optimizer.visualizeOptimizationResult(backend_optimizer.m_raw_inverse_response);

            // Remove frames except for some overlap in order to align exposures later
            for(int k = 0;k < run_settings->nr_active_frames-30;k++)
            {
                database.removeLastFrame();
            }
        }
        else
        {
            //Something went wrong..
            std::cout << "Error: Optimization block could not be fetched." << std::endl;
            return -1;
        }
    }
    
    return 0;
}

int run_online_calibration(Settings *run_settings,std::vector<double> gt_exp_times)
{
    double vis_exponent = 1.0;

    int safe_zone_size = run_settings->nr_images_rapid_exp + 5;

    //  Set up the object to read new images from
    ImageReader image_reader(run_settings->image_folder, cv::Size(run_settings->image_width, run_settings->image_height));

    // Set up the information database
    Database database(run_settings->image_width,run_settings->image_height);

    // Setup the rapid  exposure time estimator
    RapidExposureTimeEstimator exposure_estimator(run_settings->nr_images_rapid_exp, &database);

    // Setup the nonlinear optimizer
    NonlinearOptimizer backend_optimizer(run_settings->keyframe_spacing,
                                         &database,
                                         safe_zone_size,
                                         run_settings->min_keyframes_valid,
                                         run_settings->tracker_patch_size);

    // Set up the object that handles the tracking and receives new images, extracts features
    Tracker tracker(run_settings->tracker_patch_size,run_settings->nr_active_features,run_settings->nr_pyramid_levels,&database);
    
    int optimize_cnt = 0;
    int num_images = image_reader.getNumImages();
    
    // Run over all input images, track the new image, estimate exposure time and optimize other parameters in the background
    for(int i = run_settings->start_image_index; i < num_images && (run_settings->end_image_index < 0 || i < run_settings->end_image_index); i++)
    {
        std::cout << "image: " << i << std::endl;

        // Read GT exposure time if available for this frame
        // Todo: Check if index is out of bounds, if gt exp file has not enough lines
        double gt_exp_time = -1.0;
        if(gt_exp_times.size() > 0)
        {
            gt_exp_time = gt_exp_times.at(i);
        }
        
        // If enough images are in the database, remove once all initial images for which no exposure time could be optimized
        // Since those frames will not be that good for backend optimization
        if(i == run_settings->nr_images_rapid_exp*2 + safe_zone_size)
        {
            for(int ii = 0;ii < run_settings->nr_images_rapid_exp;ii++)
            {
                database.removeLastFrame();
            }
        }
        
        // If the database is large enough, start removing old frames
        if(i > run_settings->nr_active_frames)
            database.removeLastFrame();
        
        // Read next input image
        cv::Mat new_image = image_reader.readImage(i);
        
        // Track input image (+ time the result)
        tracker.trackNewFrame(new_image,gt_exp_time);
      
        // Rapid exposure time estimation (+ time the result)
        double exposure_time = exposure_estimator.estimateExposureTime();
        database.m_tracked_frames.at(database.m_tracked_frames.size()-1).m_exp_time = exposure_time;
        database.visualizeRapidExposureTimeEstimates(vis_exponent);

        // Remove the exposure time from the radiance estimates
        std::vector<Feature*>* features = &database.m_tracked_frames.at(database.m_tracked_frames.size()-1).m_features;
        for(int k = 0;k < features->size();k++)
        {
            for(int r = 0;r < features->at(k)->m_radiance_estimates.size();r++)
            {
                features->at(k)->m_radiance_estimates.at(r) /= exposure_time;
            }
        }
  
        // Visualize tracking
        if(i%run_settings->visualize_cnt == 0)
            database.visualizeTracking();
        
        pthread_mutex_lock(&g_is_optimizing_mutex);
        bool is_optimizing = g_is_optimizing;
        pthread_mutex_unlock(&g_is_optimizing_mutex);
        
        // Optimization is still running, don't do anything and keep tracking
        if(is_optimizing)
        {
            continue;
        }
        
        //optimization is currently not running
        // (1) Fetch the current optimization result and update database
        // (2) Try to extract a new optimization block and restart optimization in the background
        
        // Fetch the old optimization result from the optimizer, if available
        if(optimize_cnt > 0)
        {
            // Write the result to the database, visualize the result
            database.m_vignette_estimate.setVignetteParameters(backend_optimizer.m_vignette_estimate);
            database.m_response_estimate.setGrossbergParameterVector(backend_optimizer.m_response_estimate);
            database.m_response_estimate.setInverseResponseVector(backend_optimizer.m_raw_inverse_response);
            
            vis_exponent = backend_optimizer.visualizeOptimizationResult(backend_optimizer.m_raw_inverse_response);
        }
        
        // Try to fetch a new optimization block
        bool succeeded = backend_optimizer.extractOptimizationBlock();
        
        if(succeeded)
        {
            // TODO: reuse thread
            //start a new optimization task here
            pthread_create(&opt_thread, NULL, run_optimization_task, (void*)&backend_optimizer);
            optimize_cnt++;
        }
    }

    pthread_join(opt_thread,NULL);

    if(optimize_cnt > 0)
    {
        // Write the result to the database, visualize the result
        database.m_vignette_estimate.setVignetteParameters(backend_optimizer.m_vignette_estimate);
        database.m_response_estimate.setGrossbergParameterVector(backend_optimizer.m_response_estimate);
        database.m_response_estimate.setInverseResponseVector(backend_optimizer.m_raw_inverse_response);

        vis_exponent = backend_optimizer.visualizeOptimizationResult(backend_optimizer.m_raw_inverse_response);
    }
    
    return 0;
}

int main(int argc, char** argv)
{
    CLI::App app("Photometric Calibration");

    Settings run_settings;
    run_settings.start_image_index   = 0;      
    run_settings.end_image_index     = -1;
    run_settings.image_width         = 640;    
    run_settings.image_height        = 480;   
    run_settings.visualize_cnt       = 1;       
    run_settings.tracker_patch_size  = 3;       
    run_settings.nr_pyramid_levels   = 2;       
    run_settings.nr_active_features  = 200;     
    run_settings.nr_images_rapid_exp = 15;     
    run_settings.image_folder = "images";
    run_settings.exposure_gt_file = "times.txt";
    run_settings.calibration_mode = "online";
    run_settings.nr_active_frames    = 200;    
    run_settings.keyframe_spacing    = 15; 
    run_settings.min_keyframes_valid = 3;

    app.add_option("-i,--image-folder", run_settings.image_folder, "Folder with image files to read.", true);
    app.add_option("--start-image-index", run_settings.start_image_index, "Start reading from this image index.", true);
    app.add_option("--end-image-index", run_settings.end_image_index, "Stop reading at this image index.", true);
    app.add_option("--image-width", run_settings.image_width, "Resize image to this width.", true);
    app.add_option("--image-height", run_settings.image_height, "Resize image to this height.", true);
    app.add_option("--exposure-gt-file", run_settings.exposure_gt_file, "Textfile containing ground truth exposure times for each frame for visualization.", true);
    app.add_option("--calibration-mode", run_settings.calibration_mode, "Choose 'online' or 'batch'", true);
    
    app.add_option("--nr-active-frames", run_settings.nr_active_frames, "Maximum number of frames to be stored in the database.", true);
    app.add_option("--keyframe-spacing", run_settings.keyframe_spacing, "Number of frames that keyframes are apart in the backend optimizer.", true);
    app.add_option("--min-keyframes-valid", run_settings.min_keyframes_valid, "Minimum number of frames a feature has to be tracked to be considered for optimization.", true);

    CLI11_PARSE(app, argc, argv);

    printf("Loading images from '%s'\n", run_settings.image_folder.c_str());
    printf("Start at index %d\n", run_settings.start_image_index);
    printf("End at index %d\n", run_settings.end_image_index);
    printf("Image width %d\n", run_settings.image_width);
    printf("Image height %d\n", run_settings.image_height);

    // Parse gt exposure times from file if available
    // Only use the last number in each line, delimiter is the space character ' '
    std::ifstream exposure_gt_file_handle(run_settings.exposure_gt_file);
    std::vector<double> gt_exp_times;
    string line;
    double min_time = 10000.0;
    double max_time = -10000.0;
    if (exposure_gt_file_handle.is_open())
    {
        while(getline(exposure_gt_file_handle,line))
        {
            std::string delimiter = " ";
            std::vector<string> split_line = split(line,' ');
            double gt_exp_time = stod(split_line.at(split_line.size()-1));
            gt_exp_times.push_back(gt_exp_time);
            if(gt_exp_time < min_time)
                min_time = gt_exp_time;
            if(gt_exp_time > max_time)
                max_time = gt_exp_time;
        }    
        std::cout << "Ground truth exposure time file successfully read (" << run_settings.exposure_gt_file << ")." << std::endl;
    }
    else
    {
        std::cout << "Ground truth exposure time file not found (" << run_settings.exposure_gt_file << ")." << std::endl;
    }
    exposure_gt_file_handle.close();

    for(int k = 0;k < gt_exp_times.size();k++)
    {    
        //normalize gt exposures to range [0,1]
        gt_exp_times.at(k) = (gt_exp_times.at(k) - min_time)/(max_time - min_time);
    }

    // Run program either in multithreaded online mode 
    // or in linearized batch mode
    if(run_settings.calibration_mode == "online")
    {
        std::cout << "Run online calibration mode." << std::endl;
        run_online_calibration(&run_settings, gt_exp_times);
    }
    else if(run_settings.calibration_mode == "batch")
    {
        std::cout << "Run batch calibration mode." << std::endl;
        run_batch_calibration(&run_settings, gt_exp_times);
    }
    else
    {
        std::cout << "Error: Unknown calibration mode '" << run_settings.calibration_mode << "'." << std::endl;
        return -1;
    }

    // wait for key-press, then exit
    std::cout << "Finished. Press key to exit." << std::endl;
    cv::waitKey(0);

    return 0;
}

