//
//  JacobianGenerator.cpp
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#include "JacobianGenerator.h"

JacobianGenerator::JacobianGenerator()
{
    // Nothing to initialize
}

void JacobianGenerator::getRawJacobianRow(double I,
                                          double r,
                                          double e,
                                          std::vector<double>& j_res,
                                          std::vector<double>& j_vig,
                                          double& j_e,
                                          double& j_I)
{
    j_res.clear();
    j_vig.clear();
    
    // Get the vignetting value
    double a2 = m_vignetting_params.at(0);
    double a4 = m_vignetting_params.at(1);
    double a6 = m_vignetting_params.at(2);
    double r2 = r * r;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double v  = 1 + a2*r2 + a4*r4 + a6*r6;
        
    // Evaluate the Grossberg base functions at the derivatives for the other derivatives
    double eIv = e * I * v;
    double h_0_d = evaluateGrossbergBaseFunction(0, true, eIv);
    double h_1_d = evaluateGrossbergBaseFunction(1, true, eIv);
    double h_2_d = evaluateGrossbergBaseFunction(2, true, eIv);
    double h_3_d = evaluateGrossbergBaseFunction(3, true, eIv);
    double h_4_d = evaluateGrossbergBaseFunction(4, true, eIv);
        
    double deriv_value = h_0_d +
                         m_response_params.at(0)*h_1_d +
                         m_response_params.at(1)*h_2_d +
                         m_response_params.at(2)*h_3_d +
                         m_response_params.at(3)*h_4_d;
    
    // Derive by the 4 Grossberg parameters
    double j_res_1  = 255*evaluateGrossbergBaseFunction(1, false, eIv);
    double j_res_2  = 255*evaluateGrossbergBaseFunction(2, false, eIv);
    double j_res_3  = 255*evaluateGrossbergBaseFunction(3, false, eIv);
    double j_res_4  = 255*evaluateGrossbergBaseFunction(4, false, eIv);
        
    j_res.push_back(j_res_1);
    j_res.push_back(j_res_2);
    j_res.push_back(j_res_3);
    j_res.push_back(j_res_4);
    
    // Derive by the 3 vignetting parameters
    double j_vig_1 = 255 * deriv_value * e * I * r2;
    double j_vig_2 = 255 * deriv_value * e * I * r4;
    double j_vig_3 = 255 * deriv_value * e * I * r6;
    
    j_vig.push_back(j_vig_1);
    j_vig.push_back(j_vig_2);
    j_vig.push_back(j_vig_3);
    
    // Derive by exposure time
    j_e = 255 * deriv_value * (I*v);

    double j_I_temp;
    getJacobianRadiance(I, r, e, j_I_temp);
    j_I = j_I_temp;
}

void JacobianGenerator::getJacobianRow_eca(double I,
                                           double r,
                                           double e,
                                           cv::Mat jacobian,
                                           int image_index,
                                           int residual_index)
{
    // Get the vignetting value
    double a2 = m_vignetting_params.at(0);
    double a4 = m_vignetting_params.at(1);
    double a6 = m_vignetting_params.at(2);
    double r2 = r * r;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double v  = 1 + a2*r2 + a4*r4 + a6*r6;
        
    // Evaluate the grossberg base functions' derivatives for the other derivatives
    double eIv = e * I * v;
    double h_0_d = evaluateGrossbergBaseFunction(0, true, eIv);
    double h_1_d = evaluateGrossbergBaseFunction(1, true, eIv);
    double h_2_d = evaluateGrossbergBaseFunction(2, true, eIv);
    double h_3_d = evaluateGrossbergBaseFunction(3, true, eIv);
    double h_4_d = evaluateGrossbergBaseFunction(4, true, eIv);

    double deriv_value = h_0_d +
                         m_response_params.at(0)*h_1_d +
                         m_response_params.at(1)*h_2_d +
                         m_response_params.at(2)*h_3_d +
                         m_response_params.at(3)*h_4_d;
        
    // Derive by the 4 Grossberg parameters
    jacobian.at<double>(residual_index,0) = 255*evaluateGrossbergBaseFunction(1, false, eIv);
    jacobian.at<double>(residual_index,1) = 255*evaluateGrossbergBaseFunction(2, false, eIv);
    jacobian.at<double>(residual_index,2) = 255*evaluateGrossbergBaseFunction(3, false, eIv);
    jacobian.at<double>(residual_index,3) = 255*evaluateGrossbergBaseFunction(4, false, eIv);
    
    // Derive by the 3 vignetting parameters
    jacobian.at<double>(residual_index,4) = 255 * deriv_value * e * I * r2;
    jacobian.at<double>(residual_index,5) = 255 * deriv_value * e * I * r4;
    jacobian.at<double>(residual_index,6) = 255 * deriv_value * e * I * r6;
        
    // Derive by exposure time
    jacobian.at<double>(residual_index,7+image_index) = 255 * deriv_value * (I*v);
}

void JacobianGenerator::getJacobianRadiance(double I,double r,double e,double& j_I)
{
    double a2 = m_vignetting_params.at(0);
    double a4 = m_vignetting_params.at(1);
    double a6 = m_vignetting_params.at(2);
    
    // Get the vignetting value
    double r2 = r * r;
    double r4 = r2 * r2;
    double r6 = r4 * r2;
    double v  = 1 + a2*r2 + a4*r4 + a6*r6;
        
    // Evaluate the Grossberg base functions' derivatives for the other derivatives
    double h_0_d = evaluateGrossbergBaseFunction(0, true, e*I*v);
    double h_1_d = evaluateGrossbergBaseFunction(1, true, e*I*v);
    double h_2_d = evaluateGrossbergBaseFunction(2, true, e*I*v);
    double h_3_d = evaluateGrossbergBaseFunction(3, true, e*I*v);
    double h_4_d = evaluateGrossbergBaseFunction(4, true, e*I*v);
        
    double deriv_value = h_0_d +
                         m_response_params.at(0)*h_1_d +
                         m_response_params.at(1)*h_2_d +
                         m_response_params.at(2)*h_3_d +
                         m_response_params.at(3)*h_4_d;
    
    j_I = 255 * deriv_value * (e*v);
}


double JacobianGenerator::evaluateGrossbergBaseFunction(int base_function_index,bool is_derivative,double x)
{
    if(x < 0)x = 0.0;
    else if(x > 1)x = 1.0;
    
    int x_int = round(x*1023);
    int x_der_int = round(x*1021);
    
    if(base_function_index == 0)
    {
        if(!is_derivative)
        {
            return m_f_0[x_int];
        }
        else
        {
            return m_f_0_der[x_der_int];
        }
    }
    
    if(base_function_index == 1)
    {
        if(!is_derivative)
        {
            return m_h_1[x_int];
        }
        else
        {
            return m_h_1_der[x_der_int];
        }
    }
    
    if(base_function_index == 2)
    {
        if(!is_derivative)
        {
            return m_h_2[x_int];
        }
        else
        {
            return m_h_2_der[x_der_int];
        }
    }
    
    if(base_function_index == 3)
    {
        if(!is_derivative)
        {
            return m_h_3[x_int];
        }
        else
        {
            return m_h_3_der[x_der_int];
        }
    }
    
    if(base_function_index == 4)
    {
        if(!is_derivative)
        {
            return m_h_4[x_int];
        }
        else
        {
            return m_h_4_der[x_der_int];
        }
    }
    
    // Error code
    return -1.0;
}

double JacobianGenerator::applyGrossbergResponse(double x)
{
    double v0 = evaluateGrossbergBaseFunction(0, false, x);
    double v1 = evaluateGrossbergBaseFunction(1, false, x);
    double v2 = evaluateGrossbergBaseFunction(2, false, x);
    double v3 = evaluateGrossbergBaseFunction(3, false, x);
    double v4 = evaluateGrossbergBaseFunction(4, false, x);
    
    double c1 = m_response_params.at(0);
    double c2 = m_response_params.at(1);
    double c3 = m_response_params.at(2);
    double c4 = m_response_params.at(3);
    
    return v0 + c1*v1 + c2*v2 + c3*v3 + c4*v4;
}

std::vector<double> JacobianGenerator::fitGrossbergModelToResponseVector(double* response)
{
    // Given a response vector, find Grossberg parameters that fit well
    cv::Mat LeftSide(4,4,CV_64F,0.0);
    cv::Mat RightSide(4,1,CV_64F,0.0);
    
    for(int i = 10;i < 240;i++)
    {
        response[i] /= 255.0;
        
        double input = i/256.0;
        
        double f0 = evaluateGrossbergBaseFunction(0, false, input);
        double f1 = evaluateGrossbergBaseFunction(1, false, input);
        double f2 = evaluateGrossbergBaseFunction(2, false, input);
        double f3 = evaluateGrossbergBaseFunction(3, false, input);
        double f4 = evaluateGrossbergBaseFunction(4, false, input);

        // For equation 1
        LeftSide.at<double>(0,0) += f1*f1;
        LeftSide.at<double>(0,1) += f1*f2;
        LeftSide.at<double>(0,2) += f1*f3;
        LeftSide.at<double>(0,3) += f1*f4;
        
        RightSide.at<double>(0,0) += (response[i]*f1 - f0*f1);
        
        // For equation 2
        LeftSide.at<double>(1,0) += f2*f1;
        LeftSide.at<double>(1,1) += f2*f2;
        LeftSide.at<double>(1,2) += f2*f3;
        LeftSide.at<double>(1,3) += f2*f4;
        
        RightSide.at<double>(1,0) += (response[i]*f2 - f0*f2);
        
        // For equation 3
        LeftSide.at<double>(2,0) += f3*f1;
        LeftSide.at<double>(2,1) += f3*f2;
        LeftSide.at<double>(2,2) += f3*f3;
        LeftSide.at<double>(2,3) += f3*f4;
        
        RightSide.at<double>(2,0) += (response[i]*f3 - f0*f3);
        
        // For equation 4
        LeftSide.at<double>(3,0) += f4*f1;
        LeftSide.at<double>(3,1) += f4*f2;
        LeftSide.at<double>(3,2) += f4*f3;
        LeftSide.at<double>(3,3) += f4*f4;
        
        RightSide.at<double>(3,0) += (response[i]*f4 - f0*f4);
    }
    
    cv::Mat Solution;
    cv::solve(LeftSide, RightSide, Solution,cv::DECOMP_SVD);
    
    std::vector<double> solution_response;
    solution_response.push_back(Solution.at<double>(0,0));
    solution_response.push_back(Solution.at<double>(1,0));
    solution_response.push_back(Solution.at<double>(2,0));
    solution_response.push_back(Solution.at<double>(3,0));
    
    return solution_response;
}
