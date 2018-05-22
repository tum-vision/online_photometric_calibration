//
//  OptimizedPoint.h
//  OnlinePhotometricCalibration
//
//  Created by Paul on 17.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#ifndef OnlineCalibration_OptimizedPoint_h
#define OnlineCalibration_OptimizedPoint_h

#include <vector>

/**
 * Struct storing the data for one optimized feature within this optimization block
 */
class OptimizedPoint
{
public:
    
    /**
     * Estimated radiances of this point (patch)
     */
    std::vector<double> radiances;
    
    /**
     * First keyframe this point is valid
     */
    int start_image_idx;
    
    /**
     * Number of keyframes this point is valid in
     */
    int num_images_valid;
    
    /**
     * Output intensities of the original input image
     */
    std::vector<std::vector<double> > output_intensities;
    
    /**
     * Normalized radii of the point in its keyframe images
     */
    std::vector<double> radii;
    
    /**
     * Gradient based weights used for optimization
     */
    std::vector<std::vector<double> > grad_weights;
    
    /**
     * x,y locations in the keyframe images
     */
    std::vector<cv::Point2f>  xy_image_locations;
        
    /**
     * Residual error for this point after optimization
     */
    double optimization_error;
};

#endif
