//
//  GainRobustTracker.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 17.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "GainRobustTracker.h"

GainRobustTracker::GainRobustTracker(int patch_size,int pyramid_levels)
{
    // Initialize patch size and pyramid levels
    m_patch_size = patch_size;
    m_pyramid_levels = pyramid_levels;
}

// Todo: change frame_1 frame 2 to ref (or const ref), pts_1 to ref
double GainRobustTracker::trackImagePyramids(cv::Mat frame_1,
                                             cv::Mat frame_2,
                                             std::vector<cv::Point2f> pts_1,
                                             std::vector<cv::Point2f>& pts_2,
                                             std::vector<int>& point_status)
{
    // All points valid in the beginning of tracking
    std::vector<int> point_validity;
    for(int i = 0;i < pts_1.size();i++)
    {
        point_validity.push_back(1);
    }
    
    // Calculate image pyramid of frame 1 and frame 2
    std::vector<cv::Mat> new_pyramid;
    cv::buildPyramid(frame_2, new_pyramid, m_pyramid_levels);
    
    std::vector<cv::Mat> old_pyramid;
    cv::buildPyramid(frame_1, old_pyramid, m_pyramid_levels);
    
    // Temporary vector to update tracking estiamtes over time
    std::vector<cv::Point2f> tracking_estimates = pts_1;
    
    double all_exp_estimates = 0.0;
    int nr_estimates = 0;
    
    // Iterate all pyramid levels and perform gain robust KLT on each level (coarse to fine)
    for(int level = (int)new_pyramid.size()-1;level >= 0;level--)
    {
        // Scale the input points and tracking estimates to the current pyramid level
        std::vector<cv::Point2f> scaled_tracked_points;
        std::vector<cv::Point2f> scaled_tracking_estimates;
        for(int i = 0;i < pts_1.size();i++)
        {
            cv::Point2f scaled_point;
            scaled_point.x = (float)(pts_1.at(i).x/pow(2,level));
            scaled_point.y = (float)(pts_1.at(i).y/pow(2,level));
            scaled_tracked_points.push_back(scaled_point);
            
            cv::Point2f scaled_estimate;
            scaled_estimate.x = (float)(tracking_estimates.at(i).x/pow(2,level));
            scaled_estimate.y = (float)(tracking_estimates.at(i).y/pow(2,level));
            scaled_tracking_estimates.push_back(scaled_estimate);
        }
        
        // Perform tracking on current level
        double exp_estimate = trackImageExposurePyr(old_pyramid.at(level),
                                                    new_pyramid.at(level),
                                                    scaled_tracked_points,
                                                    scaled_tracking_estimates,
                                                    point_validity);
        
        // Optional: Do something with the estimated exposure ratio
        // std::cout << "Estimated exposure ratio of current level: " << exp_estimate << std::endl;
        
        // Average estimates of each level later
        all_exp_estimates += exp_estimate;
        nr_estimates++;
        
        // Update the current tracking result by scaling down to pyramid level 0
        for(int i = 0;i < scaled_tracking_estimates.size();i++)
        {
            if(point_validity.at(i) == 0)
                continue;
            
            cv::Point2f scaled_point;
            scaled_point.x = (float)(scaled_tracking_estimates.at(i).x*pow(2,level));
            scaled_point.y = (float)(scaled_tracking_estimates.at(i).y*pow(2,level));
            
            tracking_estimates.at(i) = scaled_point;
        }
    }
    
    // Write result to output vectors passed by reference
    pts_2 = tracking_estimates;
    point_status = point_validity;
    
    // Average exposure ratio estimate
    double overall_exp_estimate = all_exp_estimates / nr_estimates;
    return overall_exp_estimate;
}

/**
 * For a reference on the meaning of the optimization variables and the overall concept of this function
 * refer to the photometric calibration paper 
 * introducing gain robust KLT tracking by Kim et al.
 */
 // Todo: change Mat and vector to ref
double GainRobustTracker::trackImageExposurePyr(cv::Mat old_image,
                                                cv::Mat new_image,
                                                std::vector<cv::Point2f> input_points,
                                                std::vector<cv::Point2f>& output_points,
                                                std::vector<int>& point_validity)
{
    // Number of points to track
    int nr_points = static_cast<int>(input_points.size());
    
    // Updated point locations which are updated throughout the iterations
    if(output_points.size() == 0)
    {
        output_points = input_points;
    }
    else if(output_points.size() != input_points.size())
    {
        std::cout << "ERROR - OUTPUT POINT SIZE != INPUT POINT SIZE!" << std::endl;
        return -1;
    }
    
    // Input image dimensions
    int image_rows = new_image.rows;
    int image_cols = new_image.cols;
    
    // Final exposure time estimate
    double K_total = 0.0;
    
    for(int round = 0;round < 1;round++)
    {
        // Get the currently valid points
        int nr_valid_points = getNrValidPoints(point_validity);
        
        // Allocate space for W,V matrices
        cv::Mat W(2*nr_valid_points,1,CV_64F,0.0);
        cv::Mat V(2*nr_valid_points,1,CV_64F,0.0);
        
        // Allocate space for U_INV and the original Us
        cv::Mat U_INV(2*nr_valid_points,2*nr_valid_points,CV_64F,0.0);
        std::vector<cv::Mat> Us;
        
        double lambda = 0;
        double m = 0;

        int absolute_point_index = -1;
        
        for(int p = 0;p < input_points.size();p++)
        {
            if(point_validity.at(p) == 0)
            {
                continue;
            }
            
            absolute_point_index++;
            
            // Build U matrix
            cv::Mat U(2,2, CV_64F, 0.0);
            
            // Bilinear image interpolation
            cv::Mat patch_intensities_1;
            cv::Mat patch_intensities_2;
            int absolute_patch_size = ((m_patch_size+1)*2+1);  // Todo: why m_patch_size+1?
            cv::getRectSubPix(new_image, cv::Size(absolute_patch_size,absolute_patch_size), output_points.at(p), patch_intensities_2,CV_32F);
            cv::getRectSubPix(old_image, cv::Size(absolute_patch_size,absolute_patch_size), input_points.at(p), patch_intensities_1,CV_32F);
            
            // Go through image patch around this point
            for(int r = 0; r < 2*m_patch_size+1;r++)
            {
                for(int c = 0; c < 2*m_patch_size+1;c++)
                {
                    // Fetch patch intensity values
                    double i_frame_1 = patch_intensities_1.at<float>(1+r,1+c);
                    double i_frame_2 = patch_intensities_2.at<float>(1+r,1+c);
                    
                    if(i_frame_1 < 1)
                        i_frame_1 = 1;
                    if(i_frame_2 < 1)
                        i_frame_2 = 1;
                    
                    // Estimate patch gradient values
                    double grad_1_x = (patch_intensities_1.at<float>(1+r,1+c+1) - patch_intensities_1.at<float>(1+r,1+c-1))/2;
                    double grad_1_y = (patch_intensities_1.at<float>(1+r+1,1+c) - patch_intensities_1.at<float>(1+r-1,1+c))/2;
                    
                    double grad_2_x = (patch_intensities_2.at<float>(1+r,1+c+1) - patch_intensities_2.at<float>(1+r,1+c-1))/2;
                    double grad_2_y = (patch_intensities_2.at<float>(1+r+1,1+c) - patch_intensities_2.at<float>(1+r-1,1+c))/2;
                    
                    double a = (1.0/i_frame_2)*grad_2_x + (1.0/i_frame_1)*grad_1_x;
                    double b = (1.0/i_frame_2)*grad_2_y + (1.0/i_frame_1)*grad_1_y;
                    double beta = log(i_frame_2/255.0) - log(i_frame_1/255.0);
                    
                    U.at<double>(0,0) += 0.5*a*a;
                    U.at<double>(1,0) += 0.5*a*b;
                    U.at<double>(0,1) += 0.5*a*b;
                    U.at<double>(1,1) += 0.5*b*b;
                    
                    W.at<double>(2*absolute_point_index,0)   -= a;
                    W.at<double>(2*absolute_point_index+1,0) -= b;
                    
                    V.at<double>(2*absolute_point_index,0)   -= beta*a;
                    V.at<double>(2*absolute_point_index+1,0) -= beta*b;
                    
                    lambda += 2;
                    m += 2*beta;
                }
            }
            
            //Back up U for re-substitution
            Us.push_back(U);
            
            //Invert matrix U for this point and write it to diagonal of overall U_INV matrix
            cv::Mat U_INV_p = U.inv();
            //std::cout << cv::determinant(U_INV_p) << std::endl;
            //std::cout << U_INV_p << std::endl;
            //std::cout << U << std::endl;
            
            U_INV.at<double>(2*absolute_point_index,2*absolute_point_index) = U_INV_p.at<double>(0,0);
            U_INV.at<double>(2*absolute_point_index+1,2*absolute_point_index) = U_INV_p.at<double>(1,0);
            U_INV.at<double>(2*absolute_point_index,2*absolute_point_index+1) = U_INV_p.at<double>(0,1);
            U_INV.at<double>(2*absolute_point_index+1,2*absolute_point_index+1) = U_INV_p.at<double>(1,1);
        }

        // Todo: check if opencv utilizes the sparsity of U
        //solve for the exposure
        cv::Mat K_MAT;
        cv::solve(-W.t()*U_INV*W+lambda, -W.t()*U_INV*V+m, K_MAT);
        double K = K_MAT.at<double>(0,0);
        
        //std::cout << -W.t()*U_INV*W+lambda << std::endl;
        //std::cout << -W.t()*U_INV*V+m << std::endl;
        //std::cout << K_MAT << std::endl;
        
        // Solve for the displacements
        absolute_point_index = -1;
        for(int p = 0;p < nr_points;p++)
        {
            if(point_validity.at(p) == 0)
                continue;
            
            absolute_point_index++;
            
            cv::Mat U_p = Us.at(absolute_point_index);
            cv::Mat V_p = V(cv::Rect(0,2*absolute_point_index,1,2));
            cv::Mat W_p = W(cv::Rect(0,2*absolute_point_index,1,2));
            
            cv::Mat displacement;
            cv::solve(U_p, V_p - K*W_p, displacement);
            
            //std::cout << displacement << std::endl;
            
            output_points.at(p).x += displacement.at<double>(0,0);
            output_points.at(p).y += displacement.at<double>(1,0);
            
            // Filter out this point if too close at the boundaries
            int filter_margin = 2;
            double x = output_points.at(p).x;
            double y = output_points.at(p).y;
            // Todo: the latter two should be ">=" ?
            if(x < filter_margin || y < filter_margin || x > image_cols-filter_margin || y > image_rows-filter_margin)
            {
                point_validity.at(p) = 0;
            }
        }
        
        K_total += K;
    }
    
    return exp(K_total);
}

int GainRobustTracker::getNrValidPoints(std::vector<int> validity_vector)
{
    // Simply sum up the validity vector
    int result = 0;
    for(int i = 0;i < validity_vector.size();i++)
    {
        result += validity_vector.at(i);
    }
    return result;
}
