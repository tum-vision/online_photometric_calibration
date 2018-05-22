//
//  NonlinearOptimizer.h
//  OnlinePhotometricCalibration
//
//  Created by Paul on 16.11.17.
//  Copyright (c) 2017-2018 Paul Bergmann and co-authors. All rights reserved.
//
//  See LICENSE.txt
//

#ifndef OnlinePhotometricCalibration_NonlinearOptimizer_h_
#define OnlinePhotometricCalibration_NonlinearOptimizer_h_

#include "Database.h"

#include "OptimizationBlock.h"

#include "JacobianGenerator.h"


/**
 * Implements optimization functions in order to recover photometric parameters 
 * from a data block (optimization block) of keyframes and tracked features
 * Can optimize jointly for exposure times E, vignette V, response function F (EVF optimization), fixing radiances
 * Can optimize for radiances, fixing exposure, response and vignette
 */

class NonlinearOptimizer
{
        
public:
    
    /**
     * Number of response and vignette parameters to optimize, must be 4 for response, 3 for vignette
     */
    const int C_NR_RESPONSE_PARAMS = 4;
    const int C_NR_VIGNETTE_PARAMS = 3;
    
    /**
     * Safe zone size is the number of frames in the database in the end that should not 
     * be read by the opt. backend because it still might be modified by the tracking.
     *
     * @param keyframe_spacing Number of frames that keyframes are apart (relative to all input frames)
     * @param database Handle to the database
     * @param safe_zone_size Don't take images from the first safe_zone_size frames of the database since they are still
     *        being processed by exposure optimization
     * @param min_keyframes_valid Minimum amount of keyframes a feature should be present in in order to include it to optimization
     * @param patch_size Size of tracking patch extracted around features
     */
    NonlinearOptimizer(int keyframe_spacing, Database* database,int safe_zone_size,int min_keyframes_valid,int patch_size);
    
    /**
     * Extract new optimization data from the database
     */
    bool extractOptimizationBlock();
    
    /**
     * Optimize jointly for exposure times E, Vignette V, response F
     * Keep radiance estimates fixed for tracked points
     * 
     * @param show_debug_prints Print some debug information such as reduction of the energy
     */
    double evfOptimization(bool show_debug_prints);
    
    /**
     * Optimize for irradiance values, keeping EVF estimate fixed
     */
    double radianceFullOptimization();
    
    /**
     * Read the current response + vignette estimates from the database and store them to the local variables of the object
     */
    void fetchResponseVignetteFromDatabase();
    
    /**
     * Current estimate of response and vignette parameters
     */
    std::vector<double> m_response_estimate;
    std::vector<double> m_vignette_estimate;
    
    /**
     * Show inverse response + vignette estimate
     * Returns the exponential factor that was used to align data to the ground truth
     */
    double visualizeOptimizationResult(double* inverse_response);
    
    /**
     * Handle exponential ambiguity by fixing the response in one point, also adjusting the vignette estimate
     */
    void smoothResponse();
    
    /**
     * Get the inverse response function vector without gamma optimization!
     */
    void getInverseResponseRaw(double* inverse_response_function);
    
    /**
     * Visualize passed information to the optimization block
     * @param img_width  Width of input images
     * @param img_height Height of input images
     */
    void visualizeOptimizationBlock(int img_width,int img_height)
    {
        m_optimization_block->visualizeBlockInformation(img_width,img_height);
    }
    
    /**
     * Store inverse response function here, used for fast plotting
     */
    double* m_raw_inverse_response;
    
private:
    
    /**
     * Spacing of keyframes used for optimization relative to all available input images
     */
    int m_keyframe_spacing;
    
    /**
     * Handle to database
     */
    Database* m_database;
    
    /**
     * Number of most recent images that should not be used for optimization since they are still modified by the tracker
     */
    int m_safe_zone_size;
    
    /**
     * Size of tracking patches
     */
    int m_patch_size;
    
    /**
     * Min. amount of keyframes a feature must be valid to be added to opt. block
     */
    int m_min_keyframes_valid;
    
    /**
     * Current optimization data for the optimizer
     */
    OptimizationBlock* m_optimization_block;
    
    /**
     * Compute photometric error O - f(eVL) for one point in one image
     */
    double getResidualValue(double O,double I,double r,double e);
    
    /**
     * Compute total photometric error for entire optimization data
     * 
     * @param total_error Summed up absolute residual error
     * @param avg_error Average absolute residual error per point
     */
    void getTotalResidualError(double& total_error,double& avg_error);
    
    /**
     * Evaluate the response function at x based on current estimate within the optimizer
     */
    double applyResponse(double x);
    
    /**
     * Evaluate the vignetting factor at radius r based on current estimate within the optimizer
     */
    double applyVignetting(double r);
    
    /**
     * Get the residual error for point p, subpoint r (in the patch region)
     */
    double getResidualErrorPoint(OptimizedPoint p,int r);
    
    /**
     * Determine the gamma factor that is needed to fix the current inverse response estimate at (x,y)
     */
    double determineGammaFixResponseAt(double* inverse_response,int x,double y);
    
    /**
     * Get the inverse response function vector, fixing it for a reasonable gamma factor (which is also returned)
     */
    double getInverseResponseFixGamma(double* inverse_response_function);
    
};

#endif // include guard
