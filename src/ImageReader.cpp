//
//  ImageReader.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "ImageReader.h"

ImageReader::ImageReader(std::string image_folder,
                         cv::Size new_img_size)
{
    m_img_new_size = new_img_size;
    getDir(image_folder, m_files);
    printf("ImageReader: got %d files in %s!\n", (int)m_files.size(), image_folder.c_str());
}

cv::Mat ImageReader::readImage(int image_index)
{
    // Read image from disk
    cv::Mat image = cv::imread(m_files.at(image_index), CV_LOAD_IMAGE_GRAYSCALE);
        
    if(!image.data)
    {
        std::cout << "ERROR READING IMAGE " << m_files.at(image_index) << std::endl;
        return cv::Mat();
    }
    
    // Resize input image
    cv::resize(image, image, m_img_new_size);
    
    return image;
}

int ImageReader::getDir(std::string dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dir.c_str())) == NULL)
    {
        return -1;
    }

    while ((dirp = readdir(dp)) != NULL)
    {
        std::string name = std::string(dirp->d_name);

        if(name != "." && name != "..")
            files.push_back(name);
    }

    closedir(dp);
    std::sort(files.begin(), files.end());

    if(dir.at(dir.length() - 1) != '/')
        dir = dir+"/";

    for(unsigned int i = 0; i < files.size(); i++)
    {
        if(files[i].at(0) != '/')
            files[i] = dir + files[i];
    }

    return (int)files.size();
}
