//
//  main.cpp
//  OnlineCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017 Paul Bergmann. All rights reserved.
//


// FIXME:
//  - include <> vs ""
//  - include guards
//  - update file header (authorship, license info)
//  - StandardIncludes.h
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

int main(int argc, char** argv)
{
    // TODO: move to settings struct
    // FIXME: use uint

    string image_folder("images");     // Image folder.

    int visualize_cnt       = 1;       // Visualize every visualize_cnt image (tracking + correction), rather slow.
    int start_image_index   = 0;       // Start image index.
    int end_image_index     = 100000;  // End image index.
    int image_width         = 640;     // Image width to resize to.
    int image_height        = 480;     // Image height to resize to.
    int nr_active_frames    = 200;     // Number of frames maintained in database.
    int tracker_patch_size  = 3;       // Image patch size used in tracker.
    int nr_active_features  = 200;     // Number of features maintained for each frame.
    int nr_pyramid_levels   = 2;       // Number of image pyramid levels used in tracker.
    int nr_images_rapid_exp = 15;      // Number of images for rapid exposure time estimation.
    int keyframe_spacing    = 15;      // Spacing for sampling keyframes in backend optimization.
    int min_keyframes_valid = 3;       // Minimum amount of keyframes a feature should be present to be included in optimization.

    CLI::App app("Online Photometric Calibration");

    app.add_option("-i,--image-folder", image_folder, "Folder with image files to read.");
    app.add_option("--start-image-index", start_image_index, "Start reading from this image index.");
    app.add_option("--end-image-index", end_image_index, "Stop reading at this image index.");
    app.add_option("--image-width", image_width, "Resize image to this witdth.");
    app.add_option("--image-height", image_width, "Resize image to this height.");

    // TODO: print whole configuration (add to setting struct)

    printf("Loading images from '%s'\n", image_folder.c_str());
    printf("Start at index %d\n", start_image_index);
    printf("End at index %d\n", end_image_index);
    printf("Image width %d\n", image_width);
    printf("Image height %d\n", image_height);

    CLI11_PARSE(app, argc, argv);

    int safe_zone_size = nr_images_rapid_exp + 5;

    //  Set up the object to read new images from
    ImageReader image_reader(image_folder, cv::Size(image_width, image_height));

    // Set up the information database
    Database database(image_width,image_height);

    // Setup the rapid  exposure time estimator
    RapidExposureTimeEstimator exposure_estimator(nr_images_rapid_exp, &database);

    // Setup the nonlinear optimizer
    NonlinearOptimizer backend_optimizer(keyframe_spacing,
                                         &database,
                                         safe_zone_size,
                                         min_keyframes_valid,
                                         tracker_patch_size);

    // Set up the object that handles the tracking and receives new images, extracts features
    Tracker tracker(tracker_patch_size,nr_active_features,nr_pyramid_levels,&database);
    
    int optimize_cnt = 0;
    int num_images = image_reader.getNumImages();
    
    // Run over all input images, track the new image, estimate exposure time and optimize other parameters in the background
    for(int i = start_image_index; i < num_images && i < end_image_index; i++)
    {
        std::cout << "image: " << i << std::endl;
        
        // If enough images are in the database, remove once all initial images for which no exposure time could be optimized
        // Since those frames will not be that good for backend optimization
        if(i == nr_images_rapid_exp*2 + safe_zone_size)
        {
            for(int ii = 0;ii < nr_images_rapid_exp;ii++)
            {
                database.removeLastFrame();
            }
        }
        
        // If the database is large enough, start removing old frames
        if(i > nr_active_frames)
            database.removeLastFrame();
        
        // Read next input image
        cv::Mat new_image = image_reader.readImage(i);
        
        // Track input image (+ time the result)
        tracker.trackNewFrame(new_image);
     
        // Rapid exposure time estimation (+ time the result)
        double exposure_time = exposure_estimator.estimateExposureTime();
        database.m_tracked_frames.at(database.m_tracked_frames.size()-1).m_exp_time = exposure_time;
        
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
        if(i%visualize_cnt == 0)
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
            
            backend_optimizer.visualizeOptimizationResult(backend_optimizer.m_raw_inverse_response);
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

    // FIXME: cancel and join optimization thread, visualize result

    // TODO: wait for user input, then exit
    
    return 0;
}

