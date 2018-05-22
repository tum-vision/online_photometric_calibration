//
//  VignetteModel.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "VignetteModel.h"

VignetteModel::VignetteModel(double v1,double v2,double v3,int image_width,int image_height)
{
    // Initialize vignette parameters
    m_v1 = v1;
    m_v2 = v2;
    m_v3 = v3;
    
    // Image information is necessary to compute normalized radial information from pixel coordinates
    m_image_width = image_width;
    m_image_height= image_height;
    
    // Max. non normalized image radius (center of image = center of radial vignetting assumption)
    m_max_radius = sqrt((image_width/2)*(image_width/2) + (image_height/2)*(image_height/2));
}

double VignetteModel::getNormalizedRadius(cv::Point2f xy_location)
{
    double x = xy_location.x;
    double y = xy_location.y;
    
    double x_norm = x - m_image_width/2;
    double y_norm = y - m_image_height/2;
    
    double radius = sqrt(x_norm*x_norm + y_norm*y_norm);
    radius /= m_max_radius;
    
    return radius;
}

double VignetteModel::getVignetteFactor(cv::Point2f xy_location)
{
    double r = getNormalizedRadius(xy_location);
    double r2 = r * r;
    double r4 = r2 * r2;
    double v = 1 + m_v1 * r2 + m_v2 * r4 + m_v3 * r2 * r4;
    
    return v;
}

double VignetteModel::getVignetteFactor(double norm_radius)
{
    double r = norm_radius;
    double r2 = r * r;
    double r4 = r2 * r2;
    double v = 1 + m_v1 * r2 + m_v2 * r4 + m_v3 * r2 * r4;
    
    return v;
}

void VignetteModel::setVignetteParameters(std::vector<double> vignette_model)
{
    m_v1 = vignette_model.at(0);
    m_v2 = vignette_model.at(1);
    m_v3 = vignette_model.at(2);
}


