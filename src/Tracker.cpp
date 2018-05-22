//
//  Tracker.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "Tracker.h"

Tracker::Tracker(int patch_size,int nr_active_features,int nr_pyramid_levels,Database* database)
{
    // Simply store all passed arguments to object
    m_patch_size = patch_size;
    m_max_nr_active_features = nr_active_features;
    m_nr_pyramid_levels = nr_pyramid_levels;
    m_database = database;
}

void Tracker::trackNewFrame(cv::Mat input_image,double gt_exp_time)
{
    // Compute gradient (necessary for weighting factors)
    // Todo: move to class member
    cv::Mat gradient_image;
    computeGradientImage(input_image, gradient_image);
    
    // Correct the input image based on the current response and vignette estimate (exposure time not known yet)
    // Todo: move to class member
    cv::Mat corrected_frame = input_image.clone();
    photometricallyCorrectImage(corrected_frame);
 
    // Empty database -> First frame - extract features and push them back
    if(m_database->m_tracked_frames.size() == 0)
    {
        initialFeatureExtraction(input_image,gradient_image,gt_exp_time);
        return;
    }
    
    // Database not empty
    
    // Fetch the old active feature locations together with their image
    std::vector<cv::Point2f> feature_locations = m_database->fetchActiveFeatureLocations();
    cv::Mat last_frame = m_database->fetchActiveImage();
    
    // Track the feature locations forward using gain robust KLT
    std::vector<cv::Point2f> tracked_points_new_frame;
    std::vector<unsigned char> tracked_point_status;
    std::vector<float> tracking_error_values;

    GainRobustTracker gain_robust_klt_tracker(C_KLT_PATCH_SIZE,C_NR_PYRAMID_LEVELS);
    std::vector<int> tracked_point_status_int;
    gain_robust_klt_tracker.trackImagePyramids(last_frame,
                                               input_image,
                                               feature_locations,
                                               tracked_points_new_frame,
                                               tracked_point_status_int);
    for(int i = 0;i < tracked_point_status_int.size();i++)
    {
        if(tracked_point_status_int.at(i) == 0)
        {
            tracked_point_status.push_back(0);
        }
        else
        {
            tracked_point_status.push_back(1);
        }
    }
     
    // Bidirectional tracking filter: Track points backwards and make sure its consistent
    std::vector<cv::Point2f> tracked_points_backtracking;
    std::vector<unsigned char> tracked_point_status_backtracking;
    std::vector<float> tracking_error_values_backtracking;
    GainRobustTracker gain_robust_klt_tracker_2(C_KLT_PATCH_SIZE,C_NR_PYRAMID_LEVELS);
    std::vector<int> tracked_point_status_int2;
    gain_robust_klt_tracker_2.trackImagePyramids(input_image,
                                                 last_frame,
                                                 tracked_points_new_frame,
                                                 tracked_points_backtracking,
                                                 tracked_point_status_int2);
    for(int i = 0;i < tracked_point_status_int.size();i++)
    {
        if(tracked_point_status_int.at(i) == 0)
        {
            tracked_point_status_backtracking.push_back(0);
        }
        else
        {
            tracked_point_status_backtracking.push_back(1);
        }
    }
    
    // Tracked points from backtracking and old frame should be the same -> check and filter by distance
    for(int p = 0;p < feature_locations.size();p++)
    {
        if(tracked_point_status.at(p) == 0) // Point already set invalid by forward tracking -> ignore
            continue;
        
        if(tracked_point_status_backtracking.at(p) == 0) // Invalid in backtracking -> set generally invalid
            tracked_point_status.at(p) = 0;
        
        // Valid in front + backtracked images -> calculate displacement error
        cv::Point2d d_p = feature_locations.at(p) - tracked_points_backtracking.at(p);
        double distance = sqrt(d_p.x*d_p.x + d_p.y*d_p.y);
        
        if(distance > C_FWD_BWD_TRACKING_THRESH)
        {
            tracked_point_status.at(p) = 0;
        }
    }
    
    Frame frame;
    frame.m_image = input_image;
    frame.m_image_corrected = corrected_frame;
    frame.m_gradient_image = gradient_image;
    frame.m_exp_time = 1.0;
    frame.m_gt_exp_time = gt_exp_time;

    // Reject features that have been tracked to the side of the image
    // Todo: validity_vector can be combined with tracked_point_status to one vector?
    std::vector<int> validity_vector = checkLocationValidity(tracked_points_new_frame);
    
    int nr_pushed_features = 0;
    for(int i = 0;i < feature_locations.size();i++)
    {
        // If the feature became invalid don't do anything, otherwise if its still valid, push it to the database and set the feature pointers
        if(tracked_point_status.at(i) == 0 || validity_vector.at(i) == 0)
            continue;
        
        // Feature is valid, set its data and push it back
        Feature* f = new Feature();
        // Todo: remove os, is, gs
        std::vector<double> os = bilinearInterpolateImagePatch(input_image,tracked_points_new_frame.at(i).x,tracked_points_new_frame.at(i).y);
        f->m_output_values = os;
        std::vector<double> is = bilinearInterpolateImagePatch(corrected_frame,tracked_points_new_frame.at(i).x,tracked_points_new_frame.at(i).y);
        f->m_radiance_estimates = is;
        std::vector<double> gs = bilinearInterpolateImagePatch(gradient_image, tracked_points_new_frame.at(i).x, tracked_points_new_frame.at(i).y);
        f->m_gradient_values = gs;
        f->m_xy_location = tracked_points_new_frame.at(i);
        f->m_next_feature = NULL;
        f->m_prev_feature =  m_database->m_tracked_frames.at(m_database->m_tracked_frames.size()-1).m_features.at(i);
        
        m_database->m_tracked_frames.at(m_database->m_tracked_frames.size()-1).m_features.at(i)->m_next_feature = f;
        
        frame.m_features.push_back(f);
        nr_pushed_features++;
    }
    
    m_database->m_tracked_frames.push_back(frame);
    
    // Extract new features
    std::vector<cv::Point2f> new_feature_locations = extractFeatures(input_image,m_database->fetchActiveFeatureLocations());
    std::vector<int> new_validity_vector = checkLocationValidity(new_feature_locations);
    for(int p = 0;p < new_feature_locations.size();p++)
    {
        // Skip invalid points (too close to the side of image)
        if(new_validity_vector.at(p) == 0)
            continue;
        
        // Push back new feature information
        Feature* f = new Feature();
        f->m_xy_location = new_feature_locations.at(p);
        f->m_next_feature = NULL;
        f->m_prev_feature = NULL;
        // Todo: remove os, is, gs
        std::vector<double> os = bilinearInterpolateImagePatch(input_image,new_feature_locations.at(p).x,new_feature_locations.at(p).y);
        f->m_output_values = os;
        std::vector<double> is = bilinearInterpolateImagePatch(corrected_frame,new_feature_locations.at(p).x,new_feature_locations.at(p).y);
        f->m_radiance_estimates = is;
        std::vector<double> gs = bilinearInterpolateImagePatch(gradient_image, new_feature_locations.at(p).x, new_feature_locations.at(p).y);
        f->m_gradient_values = gs;
        m_database->m_tracked_frames.at(m_database->m_tracked_frames.size()-1).m_features.push_back(f);
    }
}

// Todo: change both types to reference
std::vector<cv::Point2f> Tracker::extractFeatures(cv::Mat frame,std::vector<cv::Point2f> old_features)
{
    std::vector<cv::Point2f> new_features;

    // No new features have to be extracted
    if(old_features.size() >= m_max_nr_active_features)
    {
        return new_features;
    }
    
    int nr_features_to_extract = static_cast<int>(m_max_nr_active_features-old_features.size());
    
    // Build spatial distribution map to check where to extract features
    int cells_r = 10;
    int cells_c = 10;
    
    double im_width  = m_database->m_image_width;
    double im_height = m_database->m_image_height;
        
    int cell_height = floor(im_height / cells_r);
    int cell_width  = floor(im_width  / cells_c);

    // Todo: change to class member
    int pointDistributionMap[cells_r][cells_c];
    for(int r = 0;r < cells_r;r++)
    {
        for(int c = 0;c < cells_c;c++)
        {
            pointDistributionMap[r][c] = 0;
        }
    }
    
    // Build the point distribution map to check where features need to be extracted mostly
    for(int p = 0;p < old_features.size();p++)
    {
        double x_value = old_features.at(p).x;
        double y_value = old_features.at(p).y;
        
        int c_bin = x_value / cell_width;
        if(c_bin >= cells_c)
            c_bin = cells_c - 1;
        
        int r_bin = y_value / cell_height;
        if(r_bin >= cells_r)
            r_bin = cells_r - 1;
        
        pointDistributionMap[r_bin][c_bin]++;
    }
    
    // Identify empty cells
    std::vector<int> empty_row_indices;
    std::vector<int> empty_col_indices;
    
    for(int r = 0;r < cells_r;r++)
    {
        for(int c = 0;c < cells_c;c++)
        {
            if(pointDistributionMap[r][c] == 0)
            {
                empty_row_indices.push_back(r);
                empty_col_indices.push_back(c);
            }
        }
    }

    // Todo: empty_col_indices might be 0!!!
    // Todo: Another bad case is: only one cell is empty and all other cells have only 1 feature inside,
    // Todo: then all the features to extract will be extracted from the single empty cell.
    int points_per_cell = ceil(nr_features_to_extract / (empty_col_indices.size()*1.0));
    
    // Extract "points per cell" features from each empty cell
    for(int i = 0;i < empty_col_indices.size();i++)
    {
        // Select random cell from where to extract features
        int random_index = rand() % empty_row_indices.size();
        
        // Select row and col
        int selected_row = empty_row_indices.at(random_index);
        int selected_col = empty_col_indices.at(random_index);
        
        // Define the region of interest where to detect a feature
        cv::Rect ROI(selected_col * cell_width,selected_row * cell_height,cell_width,cell_height);
        
        // Extract features from this frame
        cv::Mat frame_roi = frame(ROI);
        
        // Extract features
        std::vector<cv::Point2f> good_corners;
        cv::goodFeaturesToTrack(frame_roi,
                                good_corners,
                                points_per_cell,
                                0.01,
                                7,
                                cv::Mat(),
                                7,
                                false,
                                0.04);
        
        // Add the strongest "points per cell" features from this extraction
        for(int k = 0;k < good_corners.size();k++)
        {
            if(k == points_per_cell)
                break;
            
            // Add the offset to the point location
            cv::Point2f point_location = good_corners.at(k);
            point_location.x += selected_col*cell_width;
            point_location.y += selected_row*cell_height;
            
            new_features.push_back(point_location);
        }
    }
    
    return new_features;
}

/**
 * Note: For this function, it is assumed that x,y lies within the image!
 */
double Tracker::bilinearInterpolateImage(cv::Mat image,double x,double y)
{
    double floor_x = std::floor(x);
    double ceil_x  = std::ceil(x);
    
    double floor_y = std::floor(y);
    double ceil_y  = std::ceil(y);
    
    // Normalize x,y to be in [0,1)
    double x_normalized = x - floor_x;
    double y_normalized = y - floor_y;
    
    // Get bilinear interpolation weights
    double w1 = (1-x_normalized)*(1-y_normalized);
    double w2 = x_normalized*(1-y_normalized);
    double w3 = (1-x_normalized)*y_normalized;
    double w4 = x_normalized*y_normalized;
    
    // Evaluate image locations
    double i1 = static_cast<double>(image.at<uchar>(floor_y,floor_x));
    double i2 = static_cast<double>(image.at<uchar>(floor_y,ceil_x));
    double i3 = static_cast<double>(image.at<uchar>(ceil_y,floor_x));
    double i4 = static_cast<double>(image.at<uchar>(ceil_y,ceil_x));
    
    // Interpolate the result
    return w1*i1 + w2*i2 + w3*i3 + w4*i4;
}

/**
 * Note: For this function, it is assumed that x,y lies within the image!
 */
std::vector<double> Tracker::bilinearInterpolateImagePatch(cv::Mat image,double x,double y)
{
    std::vector<double> result;
    
    for(int x_offset = -m_patch_size;x_offset <= m_patch_size;x_offset++)
    {
        for(int y_offset = -m_patch_size;y_offset <= m_patch_size;y_offset++)
        {
            double o_value = bilinearInterpolateImage(image,x+x_offset,y+y_offset);
            result.push_back(o_value);
        }
    }
    
    return result;
}

// Todo: change return to parameter passed by ref
std::vector<int> Tracker::checkLocationValidity(std::vector<cv::Point2f> points)
{
    // Check for each passed point location if the patch centered around it falls completely within the input images
    // Return 0 for a point if not, 1 if yes
    
    int min_x = m_patch_size+1;  //Todo: should be m_patch_size?
    int min_y = m_patch_size+1;
    
    int max_x = m_database->m_image_width-m_patch_size-1;
    int max_y = m_database->m_image_height-m_patch_size-1;
    
    std::vector<int> is_valid;
    
    for(int i = 0;i < points.size();i++)
    {
        if(points.at(i).x < min_x || points.at(i).x > max_x || points.at(i).y < min_y || points.at(i).y > max_y)
        {
            is_valid.push_back(0);
        }
        else
        {
            is_valid.push_back(1);
        }
    }
    
    return is_valid;
}

// Todo: change parameter type to reference (or const reference)
void Tracker::initialFeatureExtraction(cv::Mat input_image,cv::Mat gradient_image,double gt_exp_time)
{
    std::vector<cv::Point2f> old_f;
    std::vector<cv::Point2f> feature_locations = extractFeatures(input_image,old_f);
    std::vector<int> validity_vector = checkLocationValidity(feature_locations);
    
    // Initialize new tracking Frame
    Frame frame;
    frame.m_image = input_image;
    frame.m_image_corrected = input_image.clone();
    frame.m_exp_time = 1.0;
    frame.m_gt_exp_time = gt_exp_time;
    
    // Push back tracked feature points to the tracking Frame
    for(int p = 0;p < feature_locations.size();p++)
    {
        // Skip invalid points (too close to the side of image)
        if(validity_vector.at(p) == 0)
            continue;
        
        // Create new feature object and associate with it output intensities, gradient values, etc.
        Feature* f = new Feature();
        f->m_xy_location = feature_locations.at(p);
        f->m_next_feature = NULL;
        f->m_prev_feature = NULL;
        std::vector<double> os = bilinearInterpolateImagePatch(input_image,feature_locations.at(p).x,feature_locations.at(p).y);
        std::vector<double> gs = bilinearInterpolateImagePatch(gradient_image,feature_locations.at(p).x,feature_locations.at(p).y);
        f->m_gradient_values = gs;
        f->m_output_values = os;
        f->m_radiance_estimates = os;
        frame.m_features.push_back(f);
    }
    
    m_database->m_tracked_frames.push_back(frame);
}

void Tracker::computeGradientImage(cv::Mat input_image,cv::Mat &gradient_image)
{
    // Blur the input image a little and apply discrete 3x3 sobel filter in x,y directions to obtain a gradient estimate
    // Todo: change to class member
    cv::Mat blurred_image;
    cv::GaussianBlur( input_image, blurred_image, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
    // Todo: change to class member
    cv::Mat grad_x,grad_y;
    cv::Sobel( blurred_image, grad_x, CV_16S, 1, 0, 3, 1.0, 0, cv::BORDER_DEFAULT );
    cv::Sobel( blurred_image, grad_y, CV_16S, 0, 1, 3, 1.0, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_x, grad_x );
    cv::convertScaleAbs( grad_y, grad_y );
    cv::addWeighted( grad_x, 0.5, grad_y, 0.5, 0, gradient_image );
}

void Tracker::photometricallyCorrectImage(cv::Mat &corrected_frame)
{
    for(int r = 0;r < corrected_frame.rows;r++)
    {
        for(int c = 0;c < corrected_frame.cols;c++)
        {
            int o_value = corrected_frame.at<uchar>(r,c);
            double radiance = m_database->m_response_estimate.removeResponse(o_value);
            double vig = m_database->m_vignette_estimate.getVignetteFactor(cv::Point2f(c,r));
            radiance /= vig;
            if(radiance > 255)radiance = 255;
            if(radiance < 0)radiance = 0;
            corrected_frame.at<uchar>(r,c) = (uchar)radiance;
        }
    }
    
    /*
     * For debugging: visualize the corrected frame
     *
     * cv::imshow("radiance image", corrected_frame);
     * cv::waitKey(0);
     *
     */
}

