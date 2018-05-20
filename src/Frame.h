//
//  Frame.h
//  OnlineCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017 Paul Bergmann. All rights reserved.
//
//  Commented
//

#ifndef __OnlineCalibration__Frame__
#define __OnlineCalibration__Frame__

#include "StandardIncludes.h"
#include "Feature.h"

/**
 * Stores tracking information for one entire image
 */
class Frame
{
    
public:
    
    /**
     * Original input image
     */
    cv::Mat m_image;
    
    /**
     * Photometrically corrected image
     */
    cv::Mat m_image_corrected;
    
    /**
     * Gradient information
     */
    cv::Mat m_gradient_image;
    
    /**
     * List of features present in this frame
     */
    std::vector<Feature*> m_features;
    
    /**
     * Exposure time estimate (from rapid exposure time estimation)
     */
    double m_exp_time;

    /**
    * Ground truth exposure time if available
    */
    double m_gt_exp_time;
};

#endif /* defined(__OnlineCalibration__Frame__) */
