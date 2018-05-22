//
//  RapidExposureTimeEstimator.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "RapidExposureTimeEstimator.h"

RapidExposureTimeEstimator::RapidExposureTimeEstimator(int window_size,Database* database)
{
    m_window_size = window_size;
    m_database    = database;
}

double RapidExposureTimeEstimator::estimateExposureTime()
{
    /*
     * A minimum number of m_window_size frames is required for exposure time estimation
     * Not enough frames available yet, fix exposure time to 1.0
     */
    if(m_database->m_tracked_frames.size() < m_window_size)
    {
        return 1.0;
    }
    
    /*
     * Iterate all features in the most current frame backwards for m_window_size images
     */
    std::vector<Feature*> features_last_frame = m_database->m_tracked_frames.at(m_database->m_tracked_frames.size()-1).m_features;
    
    double e_estimate = 0.0;
    double nr_estimates = 0;
    
    for(int i = 0;i < features_last_frame.size();i++)
    {
        //skip newly extracted features (no link to previous frame, cannot be used for exp. estimation)
        if(features_last_frame.at(i)->m_prev_feature == NULL)
            continue;
        
        // average the radiance estimate for this feature from the last m_window_size frames
        std::vector<double> radiances = features_last_frame.at(i)->m_prev_feature->m_radiance_estimates;
        
        // count the number of features used to estimate the point radiances
        int nr_features_used  = 1;
        
        // Iterate the rest of the m_window_size features in the other images to accumulate radiance information
        Feature* curr_feature = features_last_frame.at(i)->m_prev_feature;

        // Todo: k should start from 1?
        for(int k = 0;k < m_window_size;k++)
        {
            // Go one feature backwards
            curr_feature = curr_feature->m_prev_feature;
            
            // No tracking information anymore -> break
            if(curr_feature == NULL)
                break;
            
            // Tracking information is available
            nr_features_used++;
            
            // Accumulate radiance information
            std::vector<double> radiances_temp = curr_feature->m_radiance_estimates;
            for(int r = 0;r < radiances_temp.size();r++)
            {
                radiances.at(r) += radiances_temp.at(r);
            }
        }
        
        // Average radiance estimates for this feature
        for(int r = 0;r < radiances.size();r++)
        {
            radiances.at(r) /= nr_features_used;
        }
  
        // Image output values (corrected by vignette + response but NOT yet by exposure) taken from the current frame
        std::vector<double> outputs = features_last_frame.at(i)->m_radiance_estimates;
        
        // Fetch gradient values for the tracked feature
        std::vector<double> grad_values = features_last_frame.at(i)->m_gradient_values;
        
        // Estimate one exposure ratio for each of the tracking patch points
        for(int k = 0;k < radiances.size();k++)
        {
            // Weight the estimate depending on the gradient (high gradient = low confidence)
            double weight = 1.0 - grad_values.at(k)/125;
            if(weight < 0)weight = 0;
            
            // Avoid division by 0 for underexposed points
            if(fabs(radiances.at(k)) < 0.0001)
                continue;
            
            // Estimate the exposure ratio
            double curr_e_estimate = (outputs.at(k) / radiances.at(k));
            
            // Skip spurious implausible estimates (such as radiance = 0)
            if(curr_e_estimate < 0.001 || curr_e_estimate > 100)
                continue;
            
            // Accumulate estimation information
            e_estimate   += weight*curr_e_estimate;
            nr_estimates += weight;
        }
    }
    
    // This should not happen, just for safety
    if(nr_estimates == 0)
        return 1.0;
    
    // Average the exposure information for each patch
    // [TODO] Maybe use a more robust way to select an exposure time in the presence of severe noise
    double final_exp_estimate = e_estimate / nr_estimates;

    // Todo: this part is confusing...
    // Handle exposure time drift
    // If corrected images are highly over/underexposes, push exposure times
    // Todo: change to use ref
    cv::Mat corrected_image = m_database->m_tracked_frames.at(m_database->m_tracked_frames.size()-1).m_image_corrected;
    cv::Mat original_image  = m_database->m_tracked_frames.at(m_database->m_tracked_frames.size()-1).m_image;
    corrected_image /= final_exp_estimate;
    
    // Count the number of pixels that are under/overexposed in the corrected image
    // And not under/overexposed in the original input image
    int nr_underexposed = 0;
    int nr_overexposed  = 0;
    int nr_pixels = corrected_image.rows*corrected_image.cols;
    
    for(int r = 0;r < corrected_image.rows;r++)
    {
        for(int c = 0;c < corrected_image.cols;c++)
        {
            uchar image_value = corrected_image.at<uchar>(r,c);
            uchar orig_value  = original_image.at<uchar>(r,c);
            if(image_value < 15 && orig_value >= 30)
            {
                nr_underexposed++;
            }
            else if(image_value > 240 && orig_value <= 200)
            {
                nr_overexposed++;
            }
        }
    }
    
    double percentage_underexposed = nr_underexposed / (1.0*nr_pixels);
    double percentage_overexposed  = nr_overexposed / (1.0*nr_pixels);
    
    //std::cout << "UNDER: " << percentage_underexposed << " -- OVER: " << percentage_overexposed << std::endl;
    
    // If the amount of over/underexposed images are too large, correct the exposure time drift
    if(percentage_overexposed > 0.05)
    {
        final_exp_estimate += 0.03;
    }
    else if(percentage_underexposed > 0.03)
    {
        final_exp_estimate -= 0.05;
    }
    
    return final_exp_estimate;
}
