//
//  RapidExposureTimeEstimator.h
//  OnlineCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017 Paul Bergmann. All rights reserved.
//

#ifndef __OnlineCalibration__RapidExposureTimeEstimator__
#define __OnlineCalibration__RapidExposureTimeEstimator__

#include "StandardIncludes.h"

#include "Database.h"

class RapidExposureTimeEstimator
{
    
public:
    
    /**
     * Constructor
     * @param window_size Nuber of frames to use for exposure optimization
     * @param database Handle to the database
     */
    RapidExposureTimeEstimator(int window_size,Database* database);
    
    /**
     * Estimate a fast exposure time for the latest frame in the database
     * The tracks of the last m_window_size frames are considered for exposure time optimization
     * Response and vignette and considered fixed, based on the current estimate in the database
     */
    double estimateExposureTime();
    
private:
    
    /**
     * Handle to the database
     */
    Database* m_database;
    
    /**
     * How many frames to take into account for exposure time estimation
     */
    int m_window_size;
};

#endif /* defined(__OnlineCalibration__RapidExposureTimeEstimator__) */
