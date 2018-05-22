//
//  RapidExposureTimeEstimator.h
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#ifndef OnlinePhotometricCalibration_RapidExposureTimeEstimator_h_
#define OnlinePhotometricCalibration_RapidExposureTimeEstimator_h_

#include "StandardIncludes.h"

#include "Database.h"

class RapidExposureTimeEstimator
{
    
public:
    
    /**
     * Constructor
     * @param window_size Number of frames to use for exposure optimization
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

#endif // include guard
