//
//  NonlinearOptimizer.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "NonlinearOptimizer.h"

NonlinearOptimizer::NonlinearOptimizer(int keyframe_spacing,
                                       Database* database,
                                       int safe_zone_size,
                                       int min_keyframes_valid,
                                       int patch_size)
{
    // Initialize object parameters with passed values
    m_keyframe_spacing = keyframe_spacing;
    m_database = database;
    m_safe_zone_size = safe_zone_size;
    m_min_keyframes_valid = min_keyframes_valid;
    m_patch_size = patch_size;
    
    // No optimization data yet on initialization
    m_optimization_block = NULL;
    
    // Keep an estimate of the inverse response here for fast plotting
    m_raw_inverse_response = new double[256];
}

bool NonlinearOptimizer::extractOptimizationBlock()
{
    int nr_images_in_database = static_cast<int>(m_database->m_tracked_frames.size());

    // Not enough images in the database yet -> extraction fails
    // Todo: why 2 times?
    if(nr_images_in_database < 2*(m_keyframe_spacing*m_min_keyframes_valid)+m_safe_zone_size)
    {
        return false;
    }
    
    // Create a new optimization block, delete the old block (if an old one exists)    
    if(m_optimization_block != NULL)
    {
        delete m_optimization_block;
    }
    m_optimization_block = new OptimizationBlock(m_patch_size);
    
    int nr_images_in_block = 0;
    m_optimization_block->deleteExposureTimes();
    
    // Iterate through all images in the database (except the most current ones used for exposure optimization)
    for(int i = 0;i < nr_images_in_database - m_safe_zone_size;i++)
    {
        // Only add keyframe images
        if(i%m_keyframe_spacing == 0)
        {
            nr_images_in_block++;
            
            // Store the image inside the optimization block for later correction
            m_optimization_block->addImage(m_database->m_tracked_frames.at(i).m_image);
            
            // Push exposure time estimate (use either the one found from rapid exposure time estimation or 1.0)
            // If 1.0 is used, then all the optimization parameters should also be initialized with the same constant values (for V,f,E,L)
            m_optimization_block->pushExposureTime(m_database->m_tracked_frames.at(i).m_exp_time,m_database->m_tracked_frames.at(i).m_gt_exp_time);
            //m_optimization_block->pushExposureTime(1.0);
        }
        
        // Get all features in the current keyframe and iterate them
        // Todo: change features to pointer?
        std::vector<Feature*> features = m_database->m_tracked_frames.at(i).m_features;
        
        for(int p = 0;p < features.size();p++)
        {
            // Skip features that are not new, don't attempt at creating a new optimization point
            if(features.at(p)->m_prev_feature != NULL)
                continue;
            
            // Find out in how many and which keyframes this point is visible
            Feature* feature_iterator = features.at(p);
            std::vector<int> keyframes_valid;
            int feature_iterator_image_index = i;
            
            // Track the feature forward until either we hit NULL or the feature is tracked out of the safe zone
            while(feature_iterator != NULL && feature_iterator_image_index < nr_images_in_database-m_safe_zone_size)
            {
                if(feature_iterator_image_index%m_keyframe_spacing == 0) //feature_iterator_image_index is a keyframe image
                    keyframes_valid.push_back(feature_iterator_image_index/m_keyframe_spacing);
                
                // Check the next feature, break if the min. number of keyframes necessary for this feature has been reached
                // -> then go on to extract image information for this feature
                feature_iterator = feature_iterator->m_next_feature;
                feature_iterator_image_index++;
                
                // Break early if the feature has been identified as tracked long enough
                if(keyframes_valid.size() >= m_min_keyframes_valid)
                    break;
            }
            
            // Feature not tracked long enough
            if(keyframes_valid.size() < m_min_keyframes_valid)
                continue;
            
            // Allocate new optimization point
            OptimizedPoint opt_p;
            opt_p.start_image_idx = keyframes_valid.at(0);
            
            // Initialize vector for radiance estimates
            std::vector<double> radiance_estimate;
            for(int r = 0;r < features.at(p)->m_radiance_estimates.size();r++)
            {
                radiance_estimate.push_back(0.0);
            }
            
            // Iterate the good feature again, now completely and extract its information
            feature_iterator = features.at(p);
            feature_iterator_image_index = i;
            int nr_keyframes_valid = 0;
            
            while(feature_iterator != NULL && feature_iterator_image_index < nr_images_in_database-m_safe_zone_size)
            {
                if(feature_iterator_image_index%m_keyframe_spacing != 0) //only extract data on keyframe images
                {
                    feature_iterator = feature_iterator->m_next_feature;
                    feature_iterator_image_index++;
                    continue;
                }
                
                nr_keyframes_valid++;
                
                // Accumulate radiance estimates (obtained using the previously corrected image data)
                for(int r = 0;r < feature_iterator->m_radiance_estimates.size();r++)
                {
                    radiance_estimate.at(r) += feature_iterator->m_radiance_estimates.at(r);
                }
                
                // Initialize the estimation problem with the average of the output intensities of the original images
                // This should be exchanged with the above initialization, if exposure values = 1.0 are used for initialization
                // Assuming unit response and no vignetting (for this, the avg. of outputs is the optimal radiance estimate)
                /*for(int r = 0;r < feature_iterator->m_output_values.size();r++)
                {
                    radiance_estimate.at(r) += feature_iterator->m_output_values.at(r);
                }*/
                
                // Store output intensities to the optimization point
                opt_p.output_intensities.push_back(feature_iterator->m_output_values);
                
                // Store xy locations
                opt_p.xy_image_locations.push_back(feature_iterator->m_xy_location);
                
                // Calculate point radius
                double radius = m_database->m_vignette_estimate.getNormalizedRadius(feature_iterator->m_xy_location);
                opt_p.radii.push_back(radius);
                
                // Set gradient weights
                std::vector<double> grad_weights;
                for(int r = 0;r < feature_iterator->m_gradient_values.size();r++)
                {
                    double grad_value = feature_iterator->m_gradient_values.at(r);
                    double weight = 1.0 - (grad_value/255.0);
                    grad_weights.push_back(weight);
                }
                opt_p.grad_weights.push_back(grad_weights);
                
                feature_iterator = feature_iterator->m_next_feature;
                feature_iterator_image_index++;
            }
            
            // Average radiance estimates
            for(int r = 0;r < features.at(p)->m_radiance_estimates.size();r++)
            {
                radiance_estimate.at(r) /= 255.0*nr_keyframes_valid;
            }
            
            // Store information about in how many keyframes this feature is valid
            opt_p.num_images_valid = nr_keyframes_valid;
            
            // Store the radiance estimates to the optimization point
            opt_p.radiances = radiance_estimate;
            
            // Add point to optimization block
            m_optimization_block->addOptimizationPoint(opt_p);
        }
    }
    
    return true;
}

double NonlinearOptimizer::evfOptimization(bool show_debug_prints)
{
    // Used for calculating first order derivatives, creating the Jacobian
    JacobianGenerator jacobian_generator;
    jacobian_generator.setResponseParameters(m_response_estimate);
    jacobian_generator.setVignettingParameters(m_vignette_estimate);
    
    // Find maximum number of residuals
    int points_per_patch = pow(2*m_patch_size+1,2);
    int num_residuals = m_optimization_block->getNrResiduals();
    
    // Number of parameters to optimize for (4 response, 3 vignette + exposure times)
    int num_parameters = C_NR_RESPONSE_PARAMS + C_NR_VIGNETTE_PARAMS + m_optimization_block->getNrImages();
    
    // Initialize empty matrices for the Jacobian and the residual vector
    cv::Mat Jacobian(num_residuals,num_parameters,CV_64F,0.0);
    cv::Mat Residuals(num_residuals,1,CV_64F,0.0);
    
    // Weight matrix for the Jacobian
    cv::Mat Weights_Jacobian(num_residuals,num_parameters,CV_64F,0.0);
    
    // Fill the Jacobian
    int residual_id = -1;
    double residual_sum = 0;
    int overall_image_index = 0;
    
    std::vector<OptimizedPoint>* points_to_optimize = m_optimization_block->getOptimizedPoints();
    
    // Iterate all tracked points
    for(int p = 0;p < points_to_optimize->size();p++)
    {
        int image_start_index = points_to_optimize->at(p).start_image_idx;
        int nr_img_valid = points_to_optimize->at(p).num_images_valid;
        
        // Iterate images the point is valid
        for(int i = 0;i < nr_img_valid;i++)
        {
            double radius = points_to_optimize->at(p).radii.at(i);
            double exposure = m_optimization_block->getExposureTime(image_start_index+i);
                
            //iterate all the points in the patch
            for(int r = 0;r < points_per_patch;r++)
            {
                double grad_weight = points_to_optimize->at(p).grad_weights.at(i).at(r);
                if(grad_weight < 0.001) // Dont include a point with close to 0 weight in optimization
                    continue;
                
                double radiance = points_to_optimize->at(p).radiances.at(r);
                double o_value = points_to_optimize->at(p).output_intensities.at(i).at(r);
                
                // Avoid I = 0 which leads to NaN errors in the Jacobian, also ignore implausible radiance estimates much larger than 1
                if(radiance < 0.001 || radiance > 1.1)
                    continue;
                if(radiance > 1)
                    radiance = 1;
                
                // Count the actual number of residuals up
                residual_id++;
                
                // Fill the Jacobian row
                jacobian_generator.getJacobianRow_eca(radiance,
                                                      radius,
                                                      exposure,
                                                      Jacobian,
                                                      overall_image_index + image_start_index + i,
                                                      residual_id);
                
                // For debugging
                /*if(show_debug_prints)
                {
                    std::cout << "JACOBIAN ROW ADDED " << std::endl;
                    for(int j = 0;j < num_parameters;j++)
                    {
                        std::cout << Jacobian.at<double>(residual_id,j) << " ";
                            
                        if(Jacobian.at<double>(residual_id,j) != Jacobian.at<double>(residual_id,j))
                        {
                            std::cout << "NAN" << std::endl;
                        }
                    }
                    std::cout << std::endl;
                }*/
                    
                // Write weight values to weight matrix
                for(int k = 0;k < num_parameters;k++)
                {
                    Weights_Jacobian.at<double>(residual_id,k) = grad_weight;
                }
                    
                // Fill the residual vector
                double residual = getResidualValue(o_value, radiance, radius, exposure);
                Residuals.at<double>(residual_id,0) = grad_weight * residual;
                
                residual_sum += std::abs(grad_weight * residual);
            }
        }
    }
        
    overall_image_index += m_optimization_block->getNrImages();
    
    int real_number_of_residuals = residual_id+1;
    
    // Get only the relevant part of the Jacobian (actual number of residuals)
    Jacobian = Jacobian(cv::Rect(0,0,num_parameters,real_number_of_residuals));
    Weights_Jacobian = Weights_Jacobian(cv::Rect(0,0,num_parameters,real_number_of_residuals));
    Residuals = Residuals(cv::Rect(0,0,1,real_number_of_residuals));
    
    // Transpose the Jacobian, calculate J^T * W *J * X = - J^T * W * r
    cv::Mat Jacobian_T;
    cv::transpose(Jacobian, Jacobian_T);
    
    cv::Mat A = Jacobian_T* (Weights_Jacobian.mul(Jacobian));
    //cv::Mat A = Jacobian.t()*Jacobian;
    cv::Mat b = - Jacobian.t() * Residuals; // Todo: reuse Jacobian_T to save time?
    
    // Get the current residual before optimization in order to compare progress
    double total_error, avg_error;
    getTotalResidualError(total_error,avg_error);
    
    if(show_debug_prints)
        std::cout << "Error before ECA adjustment: total: " << total_error << " avg: " << avg_error << std::endl;
    
    // Prepare identity matrix for Levenberg-Marquardt dampening
    cv::Mat Identity = cv::Mat::eye(num_parameters, num_parameters, CV_64F);
    Identity = Identity.mul(A);
    
    // Backup photometric parameters in order to revert if update is not good
    std::vector<double> response_param_backup    = m_response_estimate;
    std::vector<double> vignetting_param_backup = m_vignette_estimate;
    std::vector<double> exp_backups;
    for(int i = 0;i < m_optimization_block->getNrImages();i++)
    {
        exp_backups.push_back(m_optimization_block->getExposureTime(i));
    }
    
    // Perform update steps
    int max_rounds = 6; // Todo: change this to const class member
    cv::Mat BestStateUpdate(num_parameters,1,CV_64F,0.0);
    double current_best_error = total_error;
    
    double lambda = 1.0f;
    double lm_dampening = 1.0;

    // Todo: are these the right LM iterations???
    // Rui: may be because of the alternative optimization of evf and radiances, thus only one iteration in the evf GN.
    for(int round = 0;round < max_rounds;round++)
    {
        if(show_debug_prints)
            std::cout << "ECA Optimization round with dampening = " << lm_dampening << std::endl;
        
        //solve the linear equation system
        cv::Mat State_Update;

        lambda = 1.0;
  
        // Solve state update equation (+LM damping)
        // Todo: reuse Jacobian_T to save time?
        State_Update = - (Jacobian.t()* Weights_Jacobian.mul(Jacobian) + lm_dampening*Identity).inv(cv::DECOMP_SVD)*(Jacobian.t()*Residuals);
        
        // Update the estimated parameters
        for(int k = 0;k < m_response_estimate.size();k++)
        {
            m_response_estimate.at(k) = response_param_backup.at(k) + lambda * State_Update.at<double>(k,0);
        }
        
        for(int k = 0;k < m_vignette_estimate.size();k++)
        {
            m_vignette_estimate.at(k) = vignetting_param_backup.at(k) + lambda * State_Update.at<double>((int)m_response_estimate.size()+k,0);
        }
        
        int abs_image_index = 0;
        for(int i = 0;i < m_optimization_block->getNrImages();i++)
        {
            double new_exp_time = exp_backups.at(i) + lambda*State_Update.at<double>((int)m_response_estimate.size()+(int)m_vignette_estimate.size()+abs_image_index,0);
            m_optimization_block->setExposureTime(i,new_exp_time);
            abs_image_index++;
        }
        
        // Evaluate new residual error with new parameter estimate
        double current_error;
        getTotalResidualError(current_error,avg_error);
        
        if(show_debug_prints)
            std::cout << "error after ECA adjustment: total: " << current_error << " avg: " << avg_error << std::endl;
        
        // Improvement?
        if(current_error < current_best_error)
        {
            //increase damping factor, re-perform
            if(lm_dampening >= 0.0625)
                lm_dampening /= 2.0f;
            
            current_best_error = current_error;
            BestStateUpdate = State_Update;
        }
        else
        {
            if(lm_dampening <= 1000000)
            {
                lm_dampening *= 2;
            }
            else
            {
                if(show_debug_prints)
                    std::cout << "MAX DAMPING REACHED, BREAK EARLY " << std::endl;
                break;
            }
        }
    }
    
    // Apply the best of the found state updates
    
    for(int k = 0;k < m_response_estimate.size();k++)
    {
        m_response_estimate.at(k) = response_param_backup.at(k) + lambda * BestStateUpdate.at<double>(k,0);
    }
    
    for(int k = 0;k < m_vignette_estimate.size();k++)
    {
        m_vignette_estimate.at(k) = vignetting_param_backup.at(k) + lambda * BestStateUpdate.at<double>((int)m_response_estimate.size()+k,0);
    }
    
    int abs_image_index = 0;
    for(int i = 0;i < m_optimization_block->getNrImages();i++)
    {
        double new_exp_time = exp_backups.at(i) +
                              lambda*BestStateUpdate.at<double>((int)m_response_estimate.size() +
                                                                (int)m_vignette_estimate.size() +
                                                                abs_image_index,0);
        m_optimization_block->setExposureTime(i,new_exp_time);
        abs_image_index++;
    }
    
    if(show_debug_prints)
        std::cout << "Best update " << BestStateUpdate << std::endl;
    
    double error_after_optimization;
    getTotalResidualError(error_after_optimization,avg_error);
    
    if(show_debug_prints)
        std::cout << "error after ECA adjustment: total: " << error_after_optimization << " avg: " << avg_error << std::endl;
    
    return avg_error;
}

double NonlinearOptimizer::getResidualValue(double O, double I, double r, double e)
{
    double vignetting = applyVignetting(r);
    
    if(vignetting < 0)
    {
        vignetting = 0;
    }
    if(vignetting > 1)
    {
        vignetting = 1;
    }
    
    // Argument of response function
    double inside = e*I*vignetting;
    double response_value;
    
    // Apply response function
    if(inside < 0)
    {
        response_value = 0;
    }
    else if(inside > 1)
    {
        response_value = 255;
    }
    else
    {
        response_value = applyResponse(inside);
    }
    
    // Compute residual
    double residual = response_value - O;
    
    return residual;
}

void NonlinearOptimizer::getTotalResidualError(double& total_error,double& avg_error)
{
    int residual_id = -1;
    double residual_sum = 0;
    int points_per_patch = pow(2*m_patch_size+1,2);
    
    std::vector<OptimizedPoint>* points_to_optimize = m_optimization_block->getOptimizedPoints();
    
    // Iterate all tracked points
    for(int p = 0;p < points_to_optimize->size();p++)
    {
        int image_start_index = points_to_optimize->at(p).start_image_idx;
        int nr_images         = points_to_optimize->at(p).num_images_valid;
        
        // Iterate all images of the point
        for(int i = 0;i < nr_images;i++)
        {
            double radius = points_to_optimize->at(p).radii.at(i);
            double exposure = m_optimization_block->getExposureTime(image_start_index+i);
                
            // Iterate all the points in the patch
            for(int r = 0;r < points_per_patch;r++)
            {
                // Handle next residual
                residual_id++;
                    
                // Get the radiance value of this residual (image independent)
                double radiance = points_to_optimize->at(p).radiances.at(r);
                
                // Get image output value of this residual (original image)
                double o_value = points_to_optimize->at(p).output_intensities.at(i).at(r);
                    
                // Compute residual
                double residual = getResidualValue(o_value, radiance, radius, exposure);
                    
                // Get the weight of the residual
                double residual_weight = points_to_optimize->at(p).grad_weights.at(i).at(r);
                
                // Accumulate resdiual values
                residual_sum += residual_weight*std::abs(residual);
            }
        }
    }
    
    // Average residual error
    total_error = residual_sum;
    avg_error   = total_error/(residual_id+1);
}

double NonlinearOptimizer::radianceFullOptimization()
{
    // Compute derivatives using this object
    JacobianGenerator jacobian_generator;
    jacobian_generator.setResponseParameters(m_response_estimate);
    jacobian_generator.setVignettingParameters(m_vignette_estimate);
    
    // Get the error before optimization
    double total_error, avg_error;
    getTotalResidualError(total_error,avg_error);
    std::cout << "error before radiance adjustment: total: " << total_error << " avg: " << avg_error << std::endl;
    
    // Nr of residuals around one tracked point
    int nr_patch_points = pow(2*m_patch_size+1,2);
    
    std::vector<OptimizedPoint>* points_to_optimize = m_optimization_block->getOptimizedPoints();
    
    // Iterate all tracked points
    for(int  p = 0;p < points_to_optimize->size();p++)
    {
        int start_image = points_to_optimize->at(p).start_image_idx;
        int num_images  = points_to_optimize->at(p).num_images_valid;
        
        // Iterate all point patches
        for(int r = 0;r < nr_patch_points;r++)
        {
            double radiance_guess = points_to_optimize->at(p).radiances.at(r);
            double left_side_sum = 0;
            double right_side_sum = 0;
            double initialResidualError = getResidualErrorPoint(points_to_optimize->at(p),r);
            
            // Iterate all images
            for(int i = 0;i < num_images;i++)
            {
                double exposure = m_optimization_block->getExposureTime(start_image+i);
                double radius   = points_to_optimize->at(p).radii.at(i);
                double output_value = points_to_optimize->at(p).output_intensities.at(i).at(r);
                    
                // Get the jacobian value
                double jacobian_I_value;
                jacobian_generator.getJacobianRadiance(radiance_guess, radius, exposure, jacobian_I_value);
                jacobian_I_value *= 255;
                    
                // Get the residual value
                double residual = getResidualValue(output_value, radiance_guess, radius, exposure);
                    
                left_side_sum += (jacobian_I_value*jacobian_I_value);
                right_side_sum += (jacobian_I_value*residual);
            }
            
            // Update radiance estimate
            double lambda = 1;
            double new_error = initialResidualError+1;
            int max_iterations = 10;
            int curr_iteration = 0;

            while(new_error > initialResidualError)
            {
                if(curr_iteration == max_iterations)
                {
                    lambda = 0;
                }

                // Todo: change to use a local variable
                points_to_optimize->at(p).radiances.at(r) = radiance_guess - lambda * (right_side_sum/left_side_sum);
                if(points_to_optimize->at(p).radiances.at(r) < 0.001)
                { 
                    points_to_optimize->at(p).radiances.at(r) = 0.001;
                }
                if(points_to_optimize->at(p).radiances.at(r) > 0.999)
                {
                    points_to_optimize->at(p).radiances.at(r) = 0.999;
                }
                    
                lambda /= 2; // Todo: to justify this in literature
                curr_iteration++;
                    
                if(lambda < 0.001)
                    break;

                new_error = getResidualErrorPoint(points_to_optimize->at(p), r);
            }
        }
    }
    
    getTotalResidualError(total_error,avg_error);
    std::cout << "error after Radiance adjustment: total: " << total_error << " avg: " << avg_error << std::endl;
    
    return avg_error;
}

double NonlinearOptimizer::getResidualErrorPoint(OptimizedPoint p,int r)
{
    int start_image = p.start_image_idx;
    int num_images  = p.num_images_valid;
    double radiance_guess = p.radiances.at(r);
    double totalError = 0;
    
    for(int i = 0;i < num_images;i++)
    {
        double radius = p.radii.at(i);
        double output_value = p.output_intensities.at(i).at(r);
        double exposure = m_optimization_block->getExposureTime(start_image+i);
        double residual = getResidualValue(output_value, radiance_guess, radius, exposure);
        totalError += std::abs(residual);
    }
    
    return totalError;
}


void NonlinearOptimizer::fetchResponseVignetteFromDatabase()
{
    // Fetch vignette estimate from database
    m_vignette_estimate = m_database->m_vignette_estimate.getVignetteEstimate();
    
    // Set response estimate to unit response
    m_response_estimate.clear();
    m_response_estimate.push_back(6.3);
    m_response_estimate.push_back(0.0);
    m_response_estimate.push_back(0.0);
    m_response_estimate.push_back(0.0);
}

double NonlinearOptimizer::applyVignetting(double r)
{
    double r_2 = r*r;
    double r_4 = r_2 * r_2;
    double r_6 = r_4 * r_2;
    
    double result_vignette = 1 + m_vignette_estimate.at(0) * r_2 + m_vignette_estimate.at(1)*r_4 + m_vignette_estimate.at(2) * r_6;
    
    if(result_vignette < 0)
        result_vignette = 0.0;
    if(result_vignette > 1)
        result_vignette = 1.0;
    
    return result_vignette;
}

double NonlinearOptimizer::applyResponse(double x)
{
    JacobianGenerator jacobian_generator;
    jacobian_generator.setResponseParameters(m_response_estimate);
    double result = jacobian_generator.applyGrossbergResponse(x);
    return 255*result;
}

double NonlinearOptimizer::visualizeOptimizationResult(double* inverse_response)
{
    // Define an exponential factor here to scale response + vignette
    // double exponent = 1.0;
    // To go through one point of the GT response of the TUM Mono camera
    //double exponent = determineGammaFixResponseAt(inverse_response, 206, 0.5);
    double exponent = determineGammaFixResponseAt(inverse_response, 148, 0.3);
    
    // Setup output image windows
    cv::namedWindow("Estimated Vignetting");
    cv::namedWindow("Estimated Response");
    
    double inverse_response_scaled[256];
    
    // Scale up inverse response from range [0,1] to range [0,255]
    for(int i = 0;i < 256;i++)
    {
        inverse_response_scaled[i] = 255*pow(inverse_response[i],exponent);
    }
    
    // Invert the inverse response to get the response
    double response_function[256];
    response_function[0] = 0;
    response_function[255] = 255;

    // For each response value i find s, such that inverse_response[s] = i
    for(int i=1;i<255;i++)
    {
        for(int s=0;s<255;s++)
        {
            if(inverse_response_scaled[s] <= i && inverse_response_scaled[s+1] >= i)
            {
                response_function[i] = s+(i - inverse_response_scaled[s]) / (inverse_response_scaled[s+1]-inverse_response_scaled[s]);
                break;
            }
        }
    }
    
    // Setup a 256x256 mat to display inverse response + response
    // Todo: change to class member
    cv::Mat response_vis_image(256,256,CV_8UC3,cv::Scalar(0,0,0));
    for(int i = 0;i < 256;i++)
    {
        int response_value = static_cast<int>(round(response_function[i]));
        int inv_response_value = static_cast<int>(round(inverse_response_scaled[i]));
        
        if(response_value < 0)
            response_value = 0;
        if(response_value > 255)
            response_value = 255;
        if(inv_response_value < 0)
            inv_response_value = 0;
        if(inv_response_value > 255)
            inv_response_value = 255;
        
        //plot the response
        response_vis_image.at<cv::Vec3b>(255-response_value,i)[0] = 0;
        response_vis_image.at<cv::Vec3b>(255-response_value,i)[1] = 0;
        response_vis_image.at<cv::Vec3b>(255-response_value,i)[2] = 255;
        
        //plot the inverse response
        //response_vis_image.at<cv::Vec3b>(255-inv_response_value,i)[0] = 0;
        //response_vis_image.at<cv::Vec3b>(255-inv_response_value,i)[1] = 0;
        //response_vis_image.at<cv::Vec3b>(255-inv_response_value,i)[2] = 255;
        
        //draw the diagonal
        //response_vis_image.at<cv::Vec3b>(255-i,i)[0] = 255;
        //response_vis_image.at<cv::Vec3b>(255-i,i)[1] = 255;
        //response_vis_image.at<cv::Vec3b>(255-i,i)[2] = 255;
        
        // Draw a variety of GT response functions
        double x = i/255.0;
        
        // [1] Draw the best gamma approximation for DSO response
        /*double dso_gamma_approx_y = pow(x,0.7-0.3*x);
         int dso_gamma_approx_y_int = static_cast<int>(dso_gamma_approx_y*255);
         if(dso_gamma_approx_y_int > 255)
             dso_gamma_approx_y_int = 255;
         response_vis_image.at<cv::Vec3b>(255-dso_gamma_approx_y_int,i)[0] = 255;
         response_vis_image.at<cv::Vec3b>(255-dso_gamma_approx_y_int,i)[1] = 255;
         response_vis_image.at<cv::Vec3b>(255-dso_gamma_approx_y_int,i)[2] = 0;*/
        
        double m;
        double t;
        //draw GT response for DSO camera
        if(x < 0.1)
        {
            m = (90/255.0)/0.1;
            t = 0.0f;
        }
        else if(x < 0.48)
        {
            m = (110.0/255)/0.38;
            t = (200.0/255.0) - m*0.48;
        }
        else
        {
            m = (55.0/255)/0.52;
            t = 1 - m*1;
        }
        double dso_value_f = m*x + t;
        int dso_value = static_cast<int>(dso_value_f*255);
        if(dso_value > 255)
            dso_value = 255;
        response_vis_image.at<cv::Vec3b>(255-dso_value,i)[0] = 255;
        response_vis_image.at<cv::Vec3b>(255-dso_value,i)[1] = 255;
        response_vis_image.at<cv::Vec3b>(255-dso_value,i)[2] = 0;
        
//        /*
//         * Draw GT response for artificial dataset
//         */
//        //draw the GT response for canon EOS 600 D
//        double artificial_value_f = pow(x,0.6-0.2*x);
//        int artificial_value = static_cast<int>(artificial_value_f*255);
//        if(artificial_value > 255)
//            artificial_value = 255;
//        response_vis_image.at<cv::Vec3b>(255-artificial_value,i)[0] = 0;
//        response_vis_image.at<cv::Vec3b>(255-artificial_value,i)[1] = 255;
//        response_vis_image.at<cv::Vec3b>(255-artificial_value,i)[2] = 255;
        
    }
    
    cv::imshow("Estimated Response", response_vis_image);
    // TODO: move only the first time the window is created,
    //       to allow the user to move it somewhere else.
    //       Same for other calls to "moveWindow".
    cv::moveWindow("Estimated Response", 20,20);

    // Show the vignetting
    
    //Setup a 256x256 mat to display vignetting
    cv::Mat vignette_vis_image(256,256,CV_8UC3,cv::Scalar(0,0,0));
    for(int i = 0;i < 256;i++)
    {
        double r = i/255.0f;
        
        double r_2 = r*r;
        double r_4 = r_2 * r_2;
        double r_6 = r_4 * r_2;
        
        double vignette = 1 + m_vignette_estimate.at(0) * r_2 + m_vignette_estimate.at(1)*r_4 + m_vignette_estimate.at(2) * r_6;
        
        vignette = pow(vignette,exponent);
        
        int y_pos = 245 - round(235*vignette);
        if(y_pos < 0)
            y_pos = 0;
        if(y_pos > 255)
            y_pos = 255;
        
        // Plot the vignetting
        vignette_vis_image.at<cv::Vec3b>(y_pos,i)[0] = 0;
        vignette_vis_image.at<cv::Vec3b>(y_pos,i)[1] = 0;
        vignette_vis_image.at<cv::Vec3b>(y_pos,i)[2] = 255;
        
        // Plot the reference line for V = 1
        //vignette_vis_image.at<cv::Vec3b>(10,i)[0] = 255;
        //vignette_vis_image.at<cv::Vec3b>(10,i)[1] = 255;
        //vignette_vis_image.at<cv::Vec3b>(10,i)[2] = 255;
        
        // Plot the reference line for V = 0
        //vignette_vis_image.at<cv::Vec3b>(235,i)[0] = 255;
        //vignette_vis_image.at<cv::Vec3b>(235,i)[1] = 255;
        //vignette_vis_image.at<cv::Vec3b>(235,i)[2] = 255;
        
        // Plot the vignetting for DSO sequence 47
         double dso_vignette_47 = 0.971 + 0.1891*r - 1.5958*r_2 + 1.4473*r_2*r - 0.5143* r_4;
         y_pos = 245 - round(235*dso_vignette_47  );
         vignette_vis_image.at<cv::Vec3b>(y_pos,i)[0] = 255;
         vignette_vis_image.at<cv::Vec3b>(y_pos,i)[1] = 255;
         vignette_vis_image.at<cv::Vec3b>(y_pos,i)[2] = 0;
        
//        // Plot the vignetting for artificial dataset
//        double art_vignette =  0.9983-0.0204*r -0.2341*r_2 - 0.0463*r_2*r;
//        y_pos = 245 - round(235*art_vignette  );
//        vignette_vis_image.at<cv::Vec3b>(y_pos,i)[0] = 0;
//        vignette_vis_image.at<cv::Vec3b>(y_pos,i)[1] = 255;
//        vignette_vis_image.at<cv::Vec3b>(y_pos,i)[2] = 255;
        
    }
    
    cv::imshow("Estimated Vignetting", vignette_vis_image);
    cv::moveWindow("Estimated Vignetting", 20,20+50+256);


    // Visualize exposure times 
    // If GT data is available, the estimated exposure times will be aligned 
    // to the GT by computing an optimal alignment factor alignment_alpha
    // If no GT data is available, the estimated exposure is simply scaled between [0,1]
    int exp_image_height = 150;
    int draw_spacing = 5;
    double alignment_alpha = 1.0;
    double max_exp = -10000.0;
    double min_exp = 10000.0;
    double top = 0;
    double bot = 0;
    int nr_block_images = m_optimization_block->getNrImages();
    std::vector<double> block_exp_estimates;
    std::vector<double> block_exp_gt;
    for(int i = 0;i < nr_block_images;i++)
    {
        double estimated_exp = m_optimization_block->getExposureTime(i);
        estimated_exp = pow(estimated_exp,exponent);
        double gt_exp = m_optimization_block->getGTExposureTime(i);

        // Store max and min exposure for normalization to [0,1] range 
        if(estimated_exp > max_exp)
            max_exp = estimated_exp;
        if(estimated_exp < min_exp)
            min_exp = estimated_exp;

        // Accumulate information for least square fit between GT and estimated exposure
        top += estimated_exp*gt_exp;
        bot += estimated_exp*estimated_exp;

        block_exp_estimates.push_back(estimated_exp);
        if(!(gt_exp < 0))
            block_exp_gt.push_back(gt_exp);
    }

    if(block_exp_estimates.size() == block_exp_gt.size())
    {
        alignment_alpha = top/bot;
    }
    else
    {
        // Normalize estimated exposures between [0,1] if no gt information is available for alignment
        for(int i = 0;i < block_exp_estimates.size();i++)
        {
            block_exp_estimates.at(i) = (block_exp_estimates.at(i)-min_exp)/(max_exp-min_exp);
        }
    }

    for(int i = 0;i < block_exp_estimates.size();i++)
    {
        block_exp_estimates.at(i) *= alignment_alpha;
    }

    // Create exposure time canvas
    draw_spacing = 20;
    int block_exp_window_width = int(fmax(400,draw_spacing*block_exp_estimates.size()));
    cv::Mat block_exposure_vis_image(exp_image_height,block_exp_window_width,CV_8UC3,cv::Scalar(0,0,0));

    // Draw estimated exposure times as lines to graph
    for(int i = 0;i < block_exp_estimates.size()-1;i++)
    {
        int drawing_y_exp_1 = exp_image_height - exp_image_height*(block_exp_estimates.at(i));
        drawing_y_exp_1 = int(fmax(0,drawing_y_exp_1));
        drawing_y_exp_1 = int(fmin(exp_image_height-1,drawing_y_exp_1));

        int drawing_y_exp_2 = exp_image_height - exp_image_height*(block_exp_estimates.at(i+1));
        drawing_y_exp_2 = int(fmax(0,drawing_y_exp_2));
        drawing_y_exp_2 = int(fmin(exp_image_height-1,drawing_y_exp_2));

        //draw exposure lines
        cv::line(block_exposure_vis_image, cv::Point(draw_spacing*i,drawing_y_exp_1), cv::Point(draw_spacing*(i+1),drawing_y_exp_2), cv::Scalar(0,0,255));
     }

    // draw GT exposure line only if GT exposure data is available
    if(block_exp_estimates.size() == block_exp_gt.size())
    {
        for(int i = 0;i < block_exp_estimates.size()-1;i++)
        {
            int drawing_y_gt_exp_1 = exp_image_height - exp_image_height * block_exp_gt.at(i);
            drawing_y_gt_exp_1 = int(fmax(0,drawing_y_gt_exp_1));
            drawing_y_gt_exp_1 = int(fmin(exp_image_height-1,drawing_y_gt_exp_1));

            int drawing_y_gt_exp_2 = exp_image_height - exp_image_height * block_exp_gt.at(i+1);
            drawing_y_gt_exp_2 = int(fmax(0,drawing_y_gt_exp_2));
            drawing_y_gt_exp_2 = int(fmin(exp_image_height-1,drawing_y_gt_exp_2));

            cv::line(block_exposure_vis_image, cv::Point(draw_spacing*i,drawing_y_gt_exp_1), cv::Point(draw_spacing*(i+1),drawing_y_gt_exp_2), cv::Scalar(255,255,0));
        }
    }   

    cv::imshow("Estimated Keyframe Exposures (Backend)", block_exposure_vis_image);
    cv::moveWindow("Estimated Keyframe Exposures (Backend)", 20+20+256,20+40+exp_image_height);


    cv::waitKey(1);

    // Return gamma that was used for visualization
    return exponent;
}

double NonlinearOptimizer::getInverseResponseFixGamma(double* inverse_response_function)
{
    getInverseResponseRaw(inverse_response_function);
    double gamma = determineGammaFixResponseAt(inverse_response_function, 127, 0.5);
    
    // Scale the inverse response
    for(int i = 0;i < 256;i++)
    {
        inverse_response_function[i] = pow(inverse_response_function[i],gamma);
    }
    
    // Return the obtained gamma factor
    return gamma;
}

void NonlinearOptimizer::getInverseResponseRaw(double* inverse_response_function)
{
    //set boundaries of the inverse response
    inverse_response_function[0] = 0;
    inverse_response_function[255] = 1.0;
    
    // For each inverse response value i find s, such that response[s] = i
    for(int i=1;i<255;i++)
    {
        bool inversion_found = false;
        
        for(int s=0;s<255;s++)
        {
            double response_s1 = applyResponse(s/255.0f);
            double response_s2 = applyResponse((s+1)/255.0f);
            if(response_s1 <= i && response_s2 >= i)
            {
                inverse_response_function[i] = s+(i - response_s1) / (response_s2-response_s1);
                inverse_response_function[i] /= 255.0;
                inversion_found = true;
                break;
            }
        }
        
        if(!inversion_found)
        {
            std::cout << "Error, no inversion found in getInverseResponse(..)" << std::endl;
        }
    }
}

double NonlinearOptimizer::determineGammaFixResponseAt(double*inverse_response,int x,double y)
{
    double v_y = inverse_response[x];
    double gamma = log(y) / log(v_y);
    return gamma;
}

void NonlinearOptimizer::smoothResponse()
{
    // Get inverse response estimate, fixing the gamma value reasonably
    double inverse_response[256];
    double gamma = getInverseResponseFixGamma(inverse_response);
    
    // Scale up the inverse response to range [0,255]
    for(int i = 0;i < 256;i++)
    {
        inverse_response[i] = 255*inverse_response[i];
    }
    
    // Invert the inverse response to get the response again
    double response_function[256];
    response_function[0] = 0;
    response_function[255] = 255;
    
    // For each response value i find s, such that inverse_response[s] = i
    for(int i=1;i<255;i++)
    {
        for(int s=0;s<255;s++)
        {
            if(inverse_response[s] <= i && inverse_response[s+1] >= i)
            {
                response_function[i] = s+(i - inverse_response[s]) / (inverse_response[s+1]-inverse_response[s]);
                break;
            }
        }
    }
    
    // Fit the Grossberg parameters new to the acquired data
    JacobianGenerator generator;
    m_response_estimate = generator.fitGrossbergModelToResponseVector(response_function);
    
    // Scale vignette by gamma factor
    double vfactors[100];
    for(int r = 0;r < 100;r++)
    {
        double radius = r/100.0;
        double vfactor = applyVignetting(radius);
        vfactor = pow(vfactor,gamma);
        vfactors[r] = vfactor;
    }
    
    //[Note] Radiances of optimization should be scaled as well, but since these are not used anymore, its not done
    int nr_block_images = m_optimization_block->getNrImages();
    for(int k = 0;k < nr_block_images;k++)
    {
        double old_exposure = m_optimization_block->getExposureTime(k);
        double new_exposure = pow(old_exposure,gamma);
        m_optimization_block->setExposureTime(k,new_exposure);
    }

    // Fit new vignetting parameters in least square manner
    cv::Mat LeftSide(3,3,CV_64F,0.0);
    cv::Mat RightSide(3,1,CV_64F,0.0);
    
    int nr_bad_v = 0;
    
    for(int r = 0;r < 100;r++)
    {
        double w = 1.0;
        if(r > 0)
        {
            double diff = vfactors[r] - vfactors[r-1];
            if(diff > 0)
            {
                w = 0.5;
                vfactors[r] = vfactors[r-1];
                nr_bad_v++;
            }
        }

        double radius = r/100.0;
        double r2 = radius*radius;
        double r4 = r2*r2;
        double r6 = r4*r2;
        double r8 = r4*r4;
        double r10 = r6*r4;
        double r12 = r6*r6;
        
        LeftSide.at<double>(0,0) += w*r4;
        LeftSide.at<double>(0,1) += w*r6;
        LeftSide.at<double>(0,2) += w*r8;
        RightSide.at<double>(0,0) += (w*vfactors[r]*r2 - w*r2);
        
        LeftSide.at<double>(1,0) += w*r6;
        LeftSide.at<double>(1,1) += w*r8;
        LeftSide.at<double>(1,2) += w*r10;
        RightSide.at<double>(1,0) += (w*vfactors[r]*r4 - w*r4);
        
        LeftSide.at<double>(2,0) += w*r8;
        LeftSide.at<double>(2,1) += w*r10;
        LeftSide.at<double>(2,2) += w*r12;
        RightSide.at<double>(2,0) += (w*vfactors[r]*r6 - w*r6);
    }
    
    cv::Mat Solution;
    cv::solve(LeftSide, RightSide, Solution, cv::DECOMP_SVD);
    
    std::vector<double> solution_vig;
    solution_vig.push_back(Solution.at<double>(0,0));
    solution_vig.push_back(Solution.at<double>(1,0));
    solution_vig.push_back(Solution.at<double>(2,0));
    
    m_vignette_estimate = solution_vig;
}
