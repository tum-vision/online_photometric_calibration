//
//  Feature.h
//  OnlineCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017 Paul Bergmann. All rights reserved.
//
//  Commented
//

#ifndef __OnlineCalibration__Feature__
#define __OnlineCalibration__Feature__

#include <stdio.h>

#include "StandardIncludes.h"

/**
 * Stores tracking information of one feature in one particular image
 */

class Feature
{
    
public:
    
    /**
     * Location of the feature in the image
     */
    cv::Point2f m_xy_location;
    
    /**
     * Output values of the image patch centered around the feature location
     */
    std::vector<double> m_output_values;
    
    /**
     * Radiance estimates of the image patch centered around the feature location
     */
    std::vector<double> m_radiance_estimates;
    
    /**
     * Gradient values of the image patch centered around the feature location
     */
    std::vector<double> m_gradient_values;
    
    /*
     * Link to the feature in the previous image corresponding to the same scene point
     * (Obtained from tracking)
     */
    Feature* m_prev_feature;
    
    /*
     * Link to the feature in the previous image corresponding to the same scene point
     * (Obtained from tracking)
     */
    Feature* m_next_feature;
    
};

#endif /* defined(__OnlineCalibration__Feature__) */
