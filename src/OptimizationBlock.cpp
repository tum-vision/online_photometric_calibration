//
//  OptimizationBlock.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 17.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "OptimizationBlock.h"

OptimizationBlock::OptimizationBlock(int patch_size)
{
    m_patch_size = patch_size;
    m_nr_patch_points =(2*m_patch_size+1)*(2*m_patch_size+1);
}

void OptimizationBlock::addOptimizationPoint(OptimizedPoint p)
{
    m_optimized_points.push_back(p);
}

/**
 * A residual arises from a tracked point location within one image, comparing the 
 * photometric image formation process to the actual output intensity
 * Residual r = O - f(e V(r) L)
 */
int OptimizationBlock::getNrResiduals()
{
    int nr_residuals = 0;
    
    for(int i = 0;i < m_optimized_points.size();i++)
    {
        int nr_images = m_optimized_points.at(i).num_images_valid;
        
        nr_residuals += (nr_images * m_nr_patch_points);
    }
    
    return nr_residuals;
}

void OptimizationBlock::visualizeBlockInformation(int image_width,int image_height)
{
    // Iterate each key frame image
    for(int i = 0;i < m_exposure_times.size();i++)
    {
        cv::Mat debug_image(image_height,image_width,CV_8UC3,cv::Scalar(0,0,0));

        // Project each tracked point patch into the debug output image
        for(int p = 0;p < m_optimized_points.size();p++)
        {
            // Is p valid in image i? if yes -> draw its output pixel information on the image
            int start_img = m_optimized_points.at(p).start_image_idx;
            int end_img = start_img + m_optimized_points.at(p).num_images_valid - 1;
            
            if(i < start_img || i > end_img)
            {
                continue;
            }
            
            // Drawing happens here
            cv::Point2f location = m_optimized_points.at(p).xy_image_locations.at(i-start_img);
            std::vector<double> outputs = m_optimized_points.at(p).output_intensities.at(i-start_img);
            
            int output_index = 0;
            for(int x = -m_patch_size;x <= m_patch_size;x++)
            {
                for(int y = -m_patch_size;y <= m_patch_size;y++)
                {
                    int x_location = (int)(location.x + x);
                    int y_location = (int)(location.y + y);
                                    
                    
                    double output = outputs.at(output_index);
                    output_index++;
                    debug_image.at<cv::Vec3b>(y_location,x_location)[0] = (uchar)output;
                    debug_image.at<cv::Vec3b>(y_location,x_location)[1] = (uchar)output;
                    debug_image.at<cv::Vec3b>(y_location,x_location)[2] = (uchar)output;
                }
            }
            
            
        }
        
        // For each pixel where no tracking information is present, draw the original image data around (in different color)
        // in order to check for consistency of the tracking data with the original images
        cv::Mat original_image = m_original_images.at(i);

        for(int r = 0;r < image_height;r++)
        {
            for(int c = 0; c < image_width;c++)
            {
                if(debug_image.at<cv::Vec3b>(r,c)[0] == 0)
                {
                    debug_image.at<cv::Vec3b>(r,c)[0] = original_image.at<uchar>(r,c);
                    debug_image.at<cv::Vec3b>(r,c)[1] = original_image.at<uchar>(r,c);
                    debug_image.at<cv::Vec3b>(r,c)[2] = 0;
                }
            }
        }
        
        cv::resize(debug_image, debug_image, cv::Size(640,480));
        
        cv::imshow("DEB", debug_image);
        cv::waitKey(0);
    }
}
