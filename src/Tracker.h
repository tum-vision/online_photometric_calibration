//
//  Tracker.h
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#ifndef OnlinePhotometricCalibration_Tracker_h_
#define OnlinePhotometricCalibration_Tracker_h_

#include "StandardIncludes.h"

#include "Database.h"

#include "GainRobustTracker.h"

#include "Frame.h"

#include "Feature.h"

/**
 *
 * Keeps original input images, corrects them based on the current V,f estimates
 *
 * Extracts features from input images and tracks them between consecutive frames 
 *
 * Implements forward backward tracking in order to filter out spurious tracks
 *
 * Uses gain robust KLT tracking to track robustly when large illumination changes between 2 frames occur
 *
 */

class Tracker
{
    
    /**
     * Forward backward tracking error threshold (if error is larger than this the track is set invalid)
     */
    const int C_FWD_BWD_TRACKING_THRESH = 2.0;
    
    /**
     * Patch size used for KLT offset optimization
     */
    const int C_KLT_PATCH_SIZE = 2;
    
    /**
     * Number of pyramid levels used
     */
    const int C_NR_PYRAMID_LEVELS = 3;
    
public:
    
    /**
     * Constructor
     * @param patch_size Size of the tracked image patches
     * @param nr_active_features Number of active features aimed to be kept for tracking
     * @param nr_pyramid_levels Number of pyramid levels to be used for KLT tracking
     * @param database Handle to the information database
     */
    Tracker(int patch_size,int nr_active_features,int nr_pyramid_levels,Database* database);
    
    /**
     * Track features from old frame to the new input frame
     * Extract new features if necessary
     * Filter out spurious tracks
     */
     // Todo: change param to reference
    void trackNewFrame(cv::Mat frame,double gt_exp_time);
    
private:
    
    /**
     * Patchsize extracted around the tracked points 
     * (not the one used for KLT tracking optimization)
     */
    int m_patch_size;
    
    /**
     * Number of active features aimed at when tracking
     */
    int m_max_nr_active_features;
    
    /**
     * Number of pyramid levels used for KLT tracking
     */
    int m_nr_pyramid_levels;
    
    /**
     * Pointer to the database to fetch and update information
     */
    Database* m_database;
    
    /**
     * Extract new features from the frame
     */
    // Todo: change param to reference
    std::vector<cv::Point2f> extractFeatures(cv::Mat frame,std::vector<cv::Point2f> old_features);
    
    /**
     * Bilinear interpolation, evaluate image at floating point location (x,y)
     */
    // Todo: change param to reference
    double bilinearInterpolateImage(cv::Mat image,double x,double y);
    
    /**
     * Evaluate entire image patch at floating point location (x,y)
     */
    // Todo: change param to reference
    std::vector<double> bilinearInterpolateImagePatch(cv::Mat image,double x,double y);
    
    /**
     * Check if points locations are not too close to the border of the image
     */
    std::vector<int> checkLocationValidity(std::vector<cv::Point2f> points);
    
    /**
     * Extract features for the first time
     */
    // Todo: change param to reference
    void initialFeatureExtraction(cv::Mat input_image,cv::Mat gradient_image,double gt_exp_time);
    
    /**
     * Compute gradient image
     */
    // Todo: change param to reference
    void computeGradientImage(cv::Mat input_image,cv::Mat &gradient_image);
    
    /**
     * Photometrically correct image based on vignette + response estimate
     */
    void photometricallyCorrectImage(cv::Mat &corrected_image);
};

#endif // include guard

