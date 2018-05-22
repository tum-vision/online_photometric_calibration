//
//  Database.h
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#ifndef OnlinePhotometricCalibration_Database_h_
#define OnlinePhotometricCalibration_Database_h_

#include "StandardIncludes.h"
#include "Frame.h"
#include "VignetteModel.h"
#include "ResponseModel.h"

class Database
{
    
public:
    
    /**
     * Initialize database
     *
     * @param image_width  Width of input images in pixels
     * @param image_height Height of input images in pixels
     */
    Database(int image_width,int image_height);
    
    /**
     * Tracked frames, exposure time estimates + radiance estimates of tracked points
     * Frames also include the tracking information + output intensities
     */
    std::vector<Frame> m_tracked_frames;
    
    /**
     * Vignette and response estimates
     */
    VignetteModel m_vignette_estimate;
    ResponseModel m_response_estimate;
    
    /**
     * Information about stored image size
     */
    int m_image_width;
    int m_image_height;
    
    /**
     * Fetch current active feature locations
     *
     * @returns Vector of feature locations active in the most current frame
     */
    std::vector<cv::Point2f> fetchActiveFeatureLocations();
    
    /**
     * Fetch most current image
     *
     * @returns Most current image in the databse
     */
     // Todo: change return type to reference
    cv::Mat fetchActiveImage()
    {
        return m_tracked_frames.at(m_tracked_frames.size()-1).m_image;
    }
    
    /**
     * Remove last frame and tracking information from the database
     */
    void removeLastFrame();
    
    /**
     * Visualize the current tracking state
     * Shows current active image with feature locations 
     * And corrected image based on current photometric estimates
     * Calling this method slows down performance of the system significantly
     */
    void visualizeTracking();

    /**
    * Visualize the rapid exposure time estimates
    */
    void visualizeRapidExposureTimeEstimates(double exponent);

};

#endif // include guard
