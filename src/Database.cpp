//
//  Database.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "Database.h"

/**
 * Initialize vignette model with some vignetting
 */
Database::Database(int image_width,int image_height) :
    m_vignette_estimate(-0.3,0,0,image_width,image_height),
    m_response_estimate()
{
    m_image_width  = image_width;
    m_image_height = image_height;
}

void Database::visualizeTracking()
{
    // Fetch tracking information and create a canvas
    Frame last_frame = m_tracked_frames.at(m_tracked_frames.size()-1);
    // Todo: change to class member
    cv::Mat draw_image = last_frame.m_image.clone();
    
    // Draw features on the canvas
    for(int i = 0;i < last_frame.m_features.size();i++)
    {
        cv::circle(draw_image, last_frame.m_features.at(i)->m_xy_location, 3, cv::Scalar(255,0,0));
    }
    
    // Correct the frame based on the current estimate
    // Todo: change to class member
    cv::Mat corrected_frame = last_frame.m_image.clone();
    
    for(int r = 0; r < corrected_frame.rows;r++)
    {
        for(int c = 0;c < corrected_frame.cols;c++)
        {
            // Apply rsponse and vignette for each pixel
            int o_value = corrected_frame.at<uchar>(r,c);
            double new_o_value = m_response_estimate.removeResponse(o_value);
            
            double v_factor = m_vignette_estimate.getVignetteFactor(cv::Point2f(c,r));
            new_o_value /= v_factor;
            
            // Correct by exposure time
            new_o_value /= last_frame.m_exp_time;
            if(new_o_value > 255)new_o_value = 255; 
            corrected_frame.at<uchar>(r,c) = (uchar)new_o_value;
        }
    }
    
    //resize drawing images to an acceptable size
    int image_width  = 640;
    int image_height = 480;
    
    cv::resize(draw_image, draw_image, cv::Size(image_width,image_height));
    cv::resize(corrected_frame, corrected_frame, cv::Size(image_width,image_height));
    
    // Display
    cv::imshow("Tracked frame", draw_image);
    cv::imshow("Corrected frame", corrected_frame);
    cv::waitKey(1);
}

void Database::visualizeRapidExposureTimeEstimates(double exponent)
{
    // Visualize exposure times 
    // If GT data is available, the estimated exposure times will be aligned 
    // to the GT by computing an optimal alignment factor alignment_alpha
    // If no GT data is available, the estimated exposure is simply scaled between [0,1]
    int nr_frames_to_vis = int(fmin(m_tracked_frames.size(),40));
    int exp_image_height = 150;
    int draw_spacing = 8;
    std::vector<double> estimated_exp_times;
    std::vector<double> gt_exp_times;
    double alignment_alpha = 1.0;
    double max_exp = -10000.0;
    double min_exp = 10000.0;
    double top = 0;
    double bot = 0;
    for(int i = 0;i < nr_frames_to_vis;i++)
    {
        // Fetch estimated and GT exposure time data, pow estimates with alignment exponent
        Frame current_frame = m_tracked_frames.at(m_tracked_frames.size()-nr_frames_to_vis+i);
        double frame_exp_time = pow(current_frame.m_exp_time,exponent);
        double frame_time_gt  = current_frame.m_gt_exp_time;

        // Keep track of max and min exposure to scale between [0,1]
        if(frame_exp_time > max_exp)
            max_exp = frame_exp_time;
        if(frame_exp_time < min_exp)
            min_exp = frame_exp_time;

        // Accumulate information for least square fit between GT and estimated exposure
        top += frame_exp_time*frame_time_gt;
        bot += frame_exp_time*frame_exp_time;
        
        // Push back estimated exposure values
        estimated_exp_times.push_back(frame_exp_time);

        // Push gt exposure time if available
        if(!(frame_time_gt < 0))
            gt_exp_times.push_back(frame_time_gt);
    }

    // Set alignment factor only if GT exposure is available
    if(gt_exp_times.size() == estimated_exp_times.size())
        alignment_alpha = top/bot;
    else
    {
        // Normalize estimated exposures between [0,1]
        for(int k = 0;k < estimated_exp_times.size();k++)
        {
            estimated_exp_times.at(k) = (estimated_exp_times.at(k)-min_exp)/(max_exp-min_exp);
        }
    }

    // Create exposure time canvas
    cv::Mat exposure_vis_image(exp_image_height,draw_spacing*nr_frames_to_vis,CV_8UC3,cv::Scalar(0,0,0));

    // Draw estimated exposure times as lines to graph
    for(int i = 0;i < nr_frames_to_vis-1;i++)
    {
        int drawing_y_exp_1 = exp_image_height - exp_image_height*(alignment_alpha * estimated_exp_times.at(i));
        drawing_y_exp_1 = int(fmax(0,drawing_y_exp_1));
        drawing_y_exp_1 = int(fmin(exp_image_height-1,drawing_y_exp_1));

        int drawing_y_exp_2 = exp_image_height - exp_image_height*(alignment_alpha * estimated_exp_times.at(i+1));
        drawing_y_exp_2 = int(fmax(0,drawing_y_exp_2));
        drawing_y_exp_2 = int(fmin(exp_image_height-1,drawing_y_exp_2));

        // Draw exposure lines
        cv::line(exposure_vis_image, cv::Point(draw_spacing*i,drawing_y_exp_1), cv::Point(draw_spacing*(i+1),drawing_y_exp_2), cv::Scalar(0,0,255));
     }

    // Draw GT exposure line only if GT exposure data is available
    if(gt_exp_times.size() == estimated_exp_times.size())
    {
        for(int i = 0;i < nr_frames_to_vis-1;i++)
        {
            int drawing_y_gt_exp_1 = exp_image_height - exp_image_height * gt_exp_times.at(i);
            drawing_y_gt_exp_1 = int(fmax(0,drawing_y_gt_exp_1));
            drawing_y_gt_exp_1 = int(fmin(exp_image_height-1,drawing_y_gt_exp_1));

            int drawing_y_gt_exp_2 = exp_image_height - exp_image_height * gt_exp_times.at(i+1);
            drawing_y_gt_exp_2 = int(fmax(0,drawing_y_gt_exp_2));
            drawing_y_gt_exp_2 = int(fmin(exp_image_height-1,drawing_y_gt_exp_2));

            cv::line(exposure_vis_image, cv::Point(draw_spacing*i,drawing_y_gt_exp_1), cv::Point(draw_spacing*(i+1),drawing_y_gt_exp_2), cv::Scalar(255,255,0));
        }
    }   

    cv::imshow("Estimated Exposure (Rapid)", exposure_vis_image);
    cv::moveWindow("Estimated Exposure (Rapid)", 20+20+256,20);

}

// Todo: change the return to parameter passed by reference
std::vector<cv::Point2f> Database::fetchActiveFeatureLocations()
{
    std::vector<cv::Point2f> point_locations;
    
    Frame last_frame = m_tracked_frames.at(m_tracked_frames.size()-1);
    
    for(int i = 0;i < last_frame.m_features.size();i++)
    {
        point_locations.push_back(last_frame.m_features.at(i)->m_xy_location);
    }
    
    return point_locations;
}

void Database::removeLastFrame()
{
    // Erase the information about the first frame
    m_tracked_frames.erase(m_tracked_frames.begin());
    
    // Let all pointers of the new first frame point to NULL
    for(int i = 0;i < m_tracked_frames.at(0).m_features.size();i++)
    {
        m_tracked_frames.at(0).m_features.at(i)->m_prev_feature = NULL;
    }
}

