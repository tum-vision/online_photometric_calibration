//
//  ImageReader.h
//  OnlineCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017 Paul Bergmann. All rights reserved.
//
//  Commented
//

#ifndef __OnlineCalibration__ImageReader__
#define __OnlineCalibration__ImageReader__

#include <stdio.h>
#include <iostream>

#include "StandardIncludes.h"

/**
 * Read input images from image files
 * Resizes the images if requested
 *
 * Images are expected to be present as number strings of constant length (m_string_size)
 * And can be read from a m_start_index up to an m_end_index
 * E.g. if m_start_index = 30, m_end_index = 120 and m_string_size = 4 then the first image to be read is
 * "0030.jpg" and the last one is "0120.jpg" (m_file_extension string being ".jpg")
 */

class ImageReader
{
public:
    
    /** 
     * Initialize the image reader
     * @param string_size Length of filename input string (before file extension)
     * @param start_index Start tracking from here
     * @param end_index End tracking at this image index
     * @param do_resize Resize input images to new_size
     * @param new_size  Resize input images to new_size
     */
    ImageReader(int string_size,std::string file_extension,int start_index,int end_index,cv::Size new_size);
    
    /*
     * Receive the next image up for tracking
     *
     * @returns Next input image up for entering the system
     */
    cv::Mat fetchNextImage();
    
private:
    
    /**
     * Length of filename input string (before file extension)
     */
    int m_string_size;
    
    /**
     * File extension string (e.g ".jpg", ".png"...)
     */
    std::string m_file_extension;
    
    /**
     * Image start index
     */
    int m_start_index;
    
    /**
     * Image end index
     */
    int m_end_index;
    
    /**
     * Resize images to this size
     */
    cv::Size m_img_new_size;
    
    /**
     * Read a new input image from the hard drive and return it
     *
     * @param Input image index to read
     * @return Read input image
     */
    cv::Mat readImage(int image_index);
    
    /**
     * Next image ID to fetch from the reader
     */
    int m_next_image_to_fetch;
};

#endif /* defined(__OnlineCalibration__ImageReader__) */
