//
//  OptimizationBlock.h
//  OnlineCalibration
//
//  Created by Paul on 17.11.17.
//  Copyright (c) 2017 Paul Bergmann. All rights reserved.
//

#ifndef __OnlineCalibration__OptimizationBlock__
#define __OnlineCalibration__OptimizationBlock__

#include "StandardIncludes.h"

#include "OptimizedPoint.h"

/**
 * An optimization block takes up the entire optimization data necessary 
 * to optimize for photometric parameters f,V,e,L in the backend.
 */

class OptimizationBlock
{
public:
    
    /**
     * Constructor, takes the patch size of the tracking patches in pixels
     */
    OptimizationBlock(int patch_size);
    
    /**
     * Add data to the optimization block
     * An optimization point corresponds to a feature tracked through several images
     * together with associated information
     */
    void addOptimizationPoint(OptimizedPoint p);
    
    /**
     * Set the exposure time estimate for image at index i
     */
    void setExposureTime(int i,double exp_time)
    {
        m_exposure_times.at(i) = exp_time;
    }
    
    /**
     * Push back a new exposure time
     */
    void pushExposureTime(double exp_time)
    {
        m_exposure_times.push_back(exp_time);
    }
    
    /**
     * Clear exposure time vector
     */
    void deleteExposureTimes()
    {
        m_exposure_times.clear();
    }
    
    /**
     * Initialize exposure times with a constant value
     */
    void initExposures(int nr_images,double init_exposure_value)
    {
        m_exposure_times.clear();
        for(int i = 0;i < nr_images;i++)
        {
            m_exposure_times.push_back(init_exposure_value);
        }
    }
    
    /**
     * Get the exposure time of image i
     */
    double getExposureTime(int i){return m_exposure_times.at(i);}
    
    /**
     * Get the number of residuals within this optimization block
     */
    int getNrResiduals();
    
    /**
     * Get the number of images within the optimization block
     */
    int getNrImages()
    {
        return static_cast<int>(m_exposure_times.size());
    }
    
    /**
     * Get optimized point information
     */
    std::vector<OptimizedPoint>* getOptimizedPoints()
    {
        return &m_optimized_points;
    }
    
    /**
     * Add the original image to the optimization block
     * (Image data only needed for later correction and visual verification of the optimization result)
     */
    void addImage(cv::Mat image)
    {
        m_original_images.push_back(image);
    }
    
    /**
     * Return the original image at index i
     */
    cv::Mat getImage(int i)
    {
        return m_original_images.at(i);
    }

    /**
     * Visualize the block (for debugging only)
     * (Shows the original images + projects the tracked image patches inside)
     */
    void visualizeBlockInformation(int image_width,int image_height);
    
    
private:
    
    /**
     * Contains all information necessary for backend optimization
     * Is extracted from the database
     */
    std::vector<OptimizedPoint> m_optimized_points;
    
    /**
     * Size of tracked patches
     */
    int m_patch_size;
    
    /**
     * Total number of points in one patch (equals (2*m_patch_size+1)^2)
     */
    int m_nr_patch_points;
    
    /**
     * Exposure time estimates of the keyframes used for backend optimization
     */
    std::vector<double> m_exposure_times;
    
    /**
     * Original images used for this block
     * (Only stored here for debugging, visual verification)
     */
    std::vector<cv::Mat> m_original_images;
};

#endif /* defined(__OnlineCalibration__OptimizationBlock__) */
