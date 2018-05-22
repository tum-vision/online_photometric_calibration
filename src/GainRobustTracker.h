//
//  GainRobustTracker.h
//  OnlinePhotometricCalibration
//
//  Created by Paul on 17.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#ifndef OnlinePhotometricCalibration_GainRobustTracker_h_
#define OnlinePhotometricCalibration_GainRobustTracker_h_

#include "StandardIncludes.h"

/**
 * This class implements gain robust KLT tracking
 * optimizing jointly for displacements of features and an exposure ratio between input frames
 */

class GainRobustTracker
{
    
public:
    
    /**
     * Constructor 
     * 
     * @param patch_size Size of tracking patches
     * @param pyramid_levels Number of pyramid levels used for KLT tracking
     */
    GainRobustTracker(int patch_size,int pyramid_levels);
    
    /*
     * Track a new image using exposure estimation + image pyramids
     *
     * @param frame_1 Frame to track points from
     * @param frame_2 Frame to track points to
     * @param pts_1 Given point locations in frame_1
     * @param pts_2 Output point locations in frame_2 (tracked from frame_1 to frame_2)
     * @param point_status Vector indicating point validity (set to 0 by tracker if e.g. tracked patches leave input images)
     * @returns Exposure ratio estimate between frame_1 and frame_2 based on KLT optimization
     */
    double trackImagePyramids(cv::Mat frame_1,
                              cv::Mat frame_2,
                              std::vector<cv::Point2f> pts_1,
                              std::vector<cv::Point2f>& pts_2,
                              std::vector<int>& point_status);
    
private:
    
    /*
     * Patch size used for tracking of image patches
     */
    int m_patch_size;
    
    /*
     * Number of pyramid levels used for tracking
     */
    int m_pyramid_levels;
    
    /**
     * Get number of valid points inside the specified validity vector
     * 
     * @param validity_vector Vector of validity flags corresponding to tracking points
     * @returns Number of valid flags inside the input vector
     */
    int getNrValidPoints(std::vector<int> validity_vector);
    
    /**
     * Track points on a specific pyramid layer
     *
     * @param old_image First input image
     * @param new_image Second input image, track new features to this image
     * @param input_points Original points in first input image
     * @param output_points Tracked point locations in second input image
     * @returns Exposure ratio estimate between first and second input image
     */
    double trackImageExposurePyr(cv::Mat old_image,
                                 cv::Mat new_image,
                                 std::vector<cv::Point2f> input_points,
                                 std::vector<cv::Point2f>& output_points,
                                 std::vector<int>& point_validity);
};

#endif // include guard
