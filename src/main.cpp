//
//  main.cpp
//  OnlineCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017 Paul Bergmann. All rights reserved.
//

#include "StandardIncludes.h"
#include "ImageReader.h"
#include "Tracker.h"
#include "RapidExposureTimeEstimator.h"
#include "Database.h"
#include "NonlinearOptimizer.h"

using namespace std;

// Optimization thread handle
pthread_t opt_thread = NULL;

// This variable indicates if currently an optimization task is running in a second thread
pthread_mutex_t g_is_optimizing_mutex;
bool g_is_optimizing = false;

void *run_optimization_task(void* thread_arg)
{
    std::cout << "START OPTIMIZATION" << std::endl;
    
    pthread_mutex_lock(&g_is_optimizing_mutex);
    g_is_optimizing = true;
    pthread_mutex_unlock(&g_is_optimizing_mutex);
    
    // The nonlinear optimzer contains all the optimization information
    NonlinearOptimizer* optimizer = (NonlinearOptimizer*)thread_arg;
    
    optimizer->fetchResponseVignetteFromDatabase();

    // Perform optimization
    optimizer->evfOptmization(false);
    
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

int main(int argc, const char * argv[]) {
    
    // Visualize every visualize_cnt image (tracking + correction)
    // Visualization is rather slow, so disabling visualization increases fps a lot
    int visualize_cnt = 1;
    
    int input_image_string_length = 5;
    string input_image_file_extension = ".png";
    
    int start_image_index = 150;
    int end_image_index = 3000;
    int num_images = end_image_index - start_image_index + 1;
    
    // Resize the input images to the below width/height
    int image_width = 640;
    int image_height = 480;
    
    // Nr of frames to store inside the database (if more frames are in database, they will be dropped)
    int nr_active_frames = 200;
    
    // Parametrize the tracker
    int tracker_patch_size = 3;
    int nr_active_features = 200;
    int nr_pyramid_levels = 2;
    
    // Parametrization of the rapid exposure time estimator
    int rapid_exp_images_to_optimize = 15;
    
    // Parametrization of the backend optimizer
    int keyframe_spacing = 15;
    int safe_zone_size = rapid_exp_images_to_optimize + 5;
    int min_keyframes_valid = 3;

    /**
     * Set up the object to read new images from
     */
    ImageReader image_reader(input_image_string_length,
                             input_image_file_extension,
                             start_image_index,
                             end_image_index,
                             cv::Size(image_width,image_height));
    
    /**
     * Set up the information database
     */
    Database database(image_width,image_height);
    
    /**
     * Setup the rapid  exposure time estimator
     */
    RapidExposureTimeEstimator exposure_estimator(rapid_exp_images_to_optimize,&database);
    
    /**
     * Setup the nonlinear optimizer
     */
    NonlinearOptimizer backend_optimizer(keyframe_spacing,
                                         &database,
                                         safe_zone_size,
                                         min_keyframes_valid,
                                         tracker_patch_size);
    
    /**
     * Set up the object that handles the tracking and receives new images, extracts features
     */
    Tracker tracker(tracker_patch_size,nr_active_features,nr_pyramid_levels,&database);
    
    int optimize_cnt = 0;
    
    // Run over all input images, track the new image, estimate exposure time and optimize other parameters in the background
    for(int i = 0;i < num_images;i++)
    {
        std::cout << "image: " << i << std::endl;
        
        // If enough images are in the database, remove once all initial images for which no exposure time could be optimized
        // Since those frames will not be that good for backend optimization
        if(i == rapid_exp_images_to_optimize*2 + safe_zone_size)
        {
            for(int i = 0;i < rapid_exp_images_to_optimize;i++)
            {
                database.removeLastFrame();
            }
        }
        
        // If the database is large enough, start removing old frames
        if(i > nr_active_frames)
            database.removeLastFrame();
        
        // Read next input image
        cv::Mat new_image = image_reader.fetchNextImage();
        
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
        
        // Fetch the old optimzation result from the optimizer, if available
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
            //start a new optimzation task here
            pthread_create(&opt_thread, NULL, run_optimization_task, (void*)&backend_optimizer);
            optimize_cnt++;
        }
    }
    
    return 0;
}

