//
//  ImageReader.cpp
//  OnlineCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017 Paul Bergmann. All rights reserved.
//

#include "ImageReader.h"

cv::Mat ImageReader::readImage(int image_index)
{
    // Padd zeros to the number string
    std::string image_number = std::to_string(image_index);
    int zeros_to_padd = m_string_size - static_cast<int>(image_number.size());
    for(int k = 0;k < zeros_to_padd;k++)
        image_number = "0" + image_number;
    
    // Read image from disk
    cv::Mat image = cv::imread(image_number + m_file_extension, CV_LOAD_IMAGE_GRAYSCALE);
        
    if(!image.data)
    {
        std::cout << "ERROR READING IMAGE " << image_index << std::endl;
        return cv::Mat();
    }
    
    // Resize input image
    cv::resize(image, image, m_img_new_size);
    
    return image;
}

ImageReader::ImageReader(int string_size,std::string file_extension,int start_index,int end_index,cv::Size new_img_size)
{
    // Simply write passed data to object
    m_string_size = string_size;
    m_file_extension = file_extension;
    m_start_index = start_index;
    m_end_index = end_index;
    m_img_new_size = new_img_size;
    m_next_image_to_fetch = m_start_index;
}

cv::Mat ImageReader::fetchNextImage()
{
    cv::Mat image = readImage(m_next_image_to_fetch);
    m_next_image_to_fetch++;
    return image;
}