//
//  FAsTMatch.cpp
//  FAsT-Match
//
//  Created by Saburo Okita on 23/05/14.
//  Copyright (c) 2014 Saburo Okita. All rights reserved.
//

#include "FAsTMatch.h"
#include <iomanip>
#include <random>
#include <tbb/tbb.h>


#define WITHIN( val, top_left, bottom_right ) (\
            val.x > top_left.x && val.y > top_left.y && \
            val.x < bottom_right.x && val.y < bottom_right.y )

namespace fast_match {
    FAsTMatch::FAsTMatch() {
        init();
    }

    void FAsTMatch::init( float epsilon, float delta, bool photometric_invariance, float min_scale, float max_scale ) {
        this->epsilon               = epsilon;
        this->delta                 = delta;
        this->photometricInvariance = photometric_invariance;
        this->minScale              = min_scale;
        this->maxScale              = max_scale;
    }

    /**
     * Apply Fast Template Matching algorithm
     */
    vector<Point2f> FAsTMatch::apply(Mat& original_image, Mat& original_template ) {
        /* Preprocess the image and template first */
        image = preprocessImage( original_image );
        templ = preprocessImage( original_template  );
        
        int r1x = 0.5 * (templ.cols - 1),
            r1y = 0.5 * (templ.rows - 1),
            r2x = 0.5 * (image.cols - 1),
            r2y = 0.5 * (image.rows - 1);
        
        float   min_trans_x  = -(r2x - r1x * minScale),
                max_trans_x  = -min_trans_x,
                min_trans_y  = -(r2y - r1y * minScale),
                max_trans_y  = -min_trans_y,
                min_rotation = -M_PI,
                max_rotation =  M_PI;
        
        /* Create the matching grid / net */
        MatchNet net( templ.cols, templ.rows, delta, min_trans_x, max_trans_x, min_trans_y, max_trans_y,
                       min_rotation, max_rotation, minScale, maxScale );
        
        /* Smooth our images */
        GaussianBlur( templ, templ, Size(0, 0), 2.0, 2.0 );
        GaussianBlur( image, image, Size(0, 0), 2.0, 2.0 );
        
        int no_of_points = round( 10 / (epsilon * epsilon) );
        
        /* Randomly sample points */
        Mat xs( 1, no_of_points, CV_32SC1 ),
            ys( 1, no_of_points, CV_32SC1 );
        
        rng.fill( xs, RNG::UNIFORM, 1, templ.cols );
        rng.fill( ys, RNG::UNIFORM, 1, templ.rows );
        
        int level = 0;
        
        float delta_fact = 1.511f;
        float new_delta  = delta;
        
        
        MatchConfig best_config;
        Mat best_trans;
        vector<double> best_distances(20, 0.0);
        double best_distance;
        vector<double> distances;
        vector<bool> insiders;
        
        while( true ) {
            level++;
            
            /* First create configurations based on our net */
            vector<MatchConfig> configs = createListOfConfigs( net, templ.size(), image.size() );
            
            int configs_count = static_cast<int>(configs.size());
            
            /* Convert the configurations into affine matrices */
            vector<Mat> affines = configsToAffine( configs, insiders );
            
            /* Filter out configurations that fall outside of the boundaries */
            /* the internal logic of configsToAffine has more information */
            vector<MatchConfig> temp_configs;
            for( int i = 0; i < insiders.size(); i++ )
                if( insiders[i] == true )
                    temp_configs.push_back( configs[i] );
            configs = temp_configs;
            
            /* For the configs, calculate the scores / distances */
            distances = evaluateConfigs( image, templ, affines, xs, ys, photometricInvariance );
            
            /* Find the minimum distance */
            auto min_itr          = min_element( distances.begin(), distances.end() );
            int min_index         = static_cast<int>(min_itr - distances.begin());
            best_distance         = distances[min_index];
            best_distances[level] = best_distance;
            
            best_config         = configs[min_index];
            best_trans          = best_config.getAffineMatrix();

            
            /* Conditions to exit the loop */
            if( (best_distance < 0.005) || ((level > 2) && (best_distance < 0.015)) || level >= 20 )
                break;
            
            if( level > 3 ) {
                float mean_value = std::accumulate( best_distances.begin() + level - 3, best_distances.begin() + level - 1, 0 ) * 1.0 / distances.size();
                
                if( best_distance > mean_value * 0.97 )
                    break;
            }
            
            
            float thresh;
            bool too_high_percentage;
            
            /* Get the good configurations that falls between certain thresholds */
            vector<MatchConfig> good_configs = getGoodConfigsByDistance( configs, best_distance, new_delta, distances, thresh, too_high_percentage );
            
            if ((too_high_percentage && (best_distance > 0.05) && ((level==1) && (configs_count < 7.5e6)) ) ||
                ((best_distance > 0.1) && ((level==1) && (configs_count < 5e6)) ) ) {
                
                static float factor = 0.9;
                new_delta    = new_delta * factor;
                level        = 0;
                net          = net * factor;
                configs      = createListOfConfigs( net, templ.size(), image.size() );
            }
            else {
                new_delta = new_delta / delta_fact;
                
                vector<MatchConfig> expanded_configs = randomExpandConfigs( good_configs, net, level, 80, delta_fact );
                
                configs.clear();
                configs.insert( configs.end(), good_configs.begin(), good_configs.end() );
                configs.insert( configs.end(), expanded_configs.begin(), expanded_configs.end() );
            }
            
            /* Randomly sample points again */
            rng.fill( xs, RNG::UNIFORM, 1, templ.cols );
            rng.fill( ys, RNG::UNIFORM, 1, templ.rows );
        }

        /* Return the rectangle corners based on the best affine transformation */
        return calcCorners( image.size(), templ.size(), best_trans );
    }

    /**
     * Given our grid / net, create a list of matching configurations
     */
    vector<MatchConfig> FAsTMatch::createListOfConfigs( MatchNet& net, Size templ_size, Size image_size ) {
        /* Creating the steps for all the parameters (i.e. translation, rotation, and scaling) */
        vector<float>   tx_steps = net.getXTranslationSteps(),
                        ty_steps = net.getYTranslationSteps(),
                        r_steps  = net.getRotationSteps(),
                        s_steps  = net.getScaleSteps();
        
        
        /* Getting the proper number of steps for each configuration parameters */
        int ntx_steps = static_cast<int>( tx_steps.size() ),
            nty_steps = static_cast<int>( ty_steps.size() ),
            ns_steps  = static_cast<int>( s_steps.size()  ),
            nr_steps  = static_cast<int>( r_steps.size()  ),
            nr2_steps = nr_steps;
        
        /* Refine the number of steps for the 2nd rotation parameter */
        if( fabs((net.boundsRotate.second - net.boundsRotate.first) - (2 * M_PI)) < 0.1 ) {
            nr2_steps = (int) count_if( r_steps.begin(), r_steps.end(), [&]( float r ){
                return r < (-M_PI / 2  + net.stepsRotate / 2);
            });
        }
        
        int grid_size = ntx_steps * nty_steps * ns_steps * ns_steps * nr_steps * nr2_steps;
        
        vector<MatchConfig> configs( grid_size );
        
        /* Iterate thru each possible affine configuration steps */
        tbb::parallel_for( 0, ntx_steps, 1, [&](int tx_index) {
            float tx = tx_steps[tx_index];
            
            for(int ty_index = 0; ty_index < nty_steps; ty_index++ ) {
                float ty = ty_steps[ty_index];
                
                for( int r1_index = 0; r1_index < nr_steps; r1_index++ ) {
                    float r1 = r_steps[r1_index];
                    
                    for( int r2_index = 0; r2_index < nr2_steps; r2_index++ ) {
                        float r2 = r_steps[r2_index];
                        
                        for( int sx_index = 0; sx_index < ns_steps; sx_index++ ) {
                            float sx = s_steps[sx_index];
                            
                            for( int sy_index = 0; sy_index < ns_steps; sy_index++ ) {
                                float sy = s_steps[sy_index];
                                
                                /* Maybe there's a better way for indexing when multithreading ... */
                                int grid_index  = (tx_index * nty_steps * nr_steps * nr2_steps * ns_steps * ns_steps)
                                                + (ty_index * nr_steps  * nr2_steps * ns_steps  * ns_steps)
                                                + (r1_index * nr2_steps * ns_steps * ns_steps)
                                                + (r2_index * ns_steps  * ns_steps)
                                                + (sx_index * ns_steps)
                                                + sy_index;

                                configs[grid_index].init( tx, ty, r2, sx, sy, r1 );
                            }
                        }
                    }
                }
            }
        });
        
        
        return configs;
    }

    /**
     * Randomly expands the configuration
     */
    vector<MatchConfig> FAsTMatch::randomExpandConfigs( vector<MatchConfig>& configs, MatchNet& net, int level,
                                                int no_of_points, float delta_factor ) {
        
        float factor = pow(delta_factor, level);
        
        float   half_step_tx = net.stepsTransX / factor,
                half_step_ty = net.stepsTransY / factor,
                half_step_r  = net.stepsRotate / factor,
                half_step_s  = net.stepsScale / factor;
        
        int no_of_configs = static_cast<int>( configs.size() );
        
        /* Create random vectors that contain values which are either -1, 0, or 1 */
        Mat random_vec( no_of_points * no_of_configs, 6, CV_32SC1 );
        rng.fill( random_vec, RNG::NORMAL, 0, 0.5 );
        random_vec.convertTo( random_vec, CV_32FC1 );

        /* Convert our vector of configurations into a large matrix */
        vector<Mat> configs_mat(no_of_configs);
        for(int i = 0; i < no_of_configs; i++ )
            configs_mat[i] = configs[i].asMatrix();
        
        Mat expanded;
        vconcat( configs_mat, expanded );
        expanded = repeat( expanded, no_of_points, 1 );

        vector<float> ranges_vec = {
            half_step_tx, half_step_ty, half_step_r, half_step_s, half_step_s, half_step_r
        };
        
        Mat ranges = repeat(Mat(ranges_vec).t() , no_of_points * no_of_configs, 1);
        
        /* The expanded configs is the original configs plus some random changes */
        Mat expanded_configs = expanded + random_vec.mul( ranges );

        return MatchConfig::fromMatrix( expanded_configs );
    }


    /**
     * From given list of configurations, convert them into affine matrices.
     * But filter out all the rectangles that are out of the given boundaries.
     **/
    vector<Mat> FAsTMatch::configsToAffine( vector<MatchConfig>& configs, vector<bool>& insiders ) {
        int no_of_configs = static_cast<int>(configs.size());
        vector<Mat> affines( no_of_configs );
        
        /* The boundary, between -10 to image size + 10 */
        Point2d top_left( -10., -10. );
        Point2d bottom_right( image.cols + 10, image.rows + 10 );
        
        
        /* These are for the calculations of affine transformed corners */
        int r1x  = 0.5 * ( templ.cols  - 1),
            r1y  = 0.5 * ( templ.rows - 1),
            r2x  = 0.5 * ( image.cols  - 1),
            r2y  = 0.5 * ( image.rows - 1);
        
        Mat corners = (Mat_<float>(3, 4) <<
                       1-(r1x+1), templ.cols-(r1x+1), templ.cols-(r1x+1),  1-(r1x+1),
                       1-(r1y+1), 1-(r1y+1)         , templ.rows-(r1y+1), templ.rows-(r1y+1),
                       1.0      , 1.0               , 1.0               , 1.0 );
        
        Mat transl = (Mat_<float>(4, 2) <<
                      r2x + 1, r2y + 1,
                      r2x + 1, r2y + 1,
                      r2x + 1, r2y + 1,
                      r2x + 1, r2y + 1 );
        
        insiders.assign( no_of_configs, false );
        
        /* Convert each configuration to corresponding affine transformation matrix */
        tbb::parallel_for( 0, no_of_configs, 1, [&](int i) {
            Mat affine = configs[i].getAffineMatrix();
            
            /* Check if our affine transformed rectangle still fits within our boundary */
            Mat affine_corners = (affine * corners).t();
            affine_corners =  affine_corners + transl;
            
            if( WITHIN( affine_corners.at<Point2f>(0), top_left, bottom_right) &&
                WITHIN( affine_corners.at<Point2f>(1), top_left, bottom_right) &&
                WITHIN( affine_corners.at<Point2f>(2), top_left, bottom_right) &&
                WITHIN( affine_corners.at<Point2f>(3), top_left, bottom_right) ) {
                
                affines[i]  = affine;
                insiders[i] = true;
            }
        });
        
        /* Filter out empty affine matrices (which initially don't fit within the preset boundary) */
        /* It's done this way, so that I could parallelize the loop */
        vector<Mat> result;
        for( int i = 0; i < no_of_configs; i++ ) {
            if( insiders[i] )
                result.push_back( affines[i] );
        }
        
        return result;
    }


    /**
     * Evaluate the score of the given configurations
     */
    vector<double> FAsTMatch::evaluateConfigs( Mat& image, Mat& templ, vector<Mat>& affine_matrices,
                                               Mat& xs, Mat& ys, bool photometric_invariance ) {
        
        int r1x = 0.5 * (templ.cols - 1),
            r1y = 0.5 * (templ.rows - 1),
            r2x = 0.5 * (image.cols - 1),
            r2y = 0.5 * (image.rows - 1);
        
        int no_of_configs = static_cast<int>(affine_matrices.size());
        int no_of_points  = xs.cols;
        
        /* Use a padded image, to avoid boundary checking */
        Mat padded( image.rows * 3, image.cols, image.type(), Scalar(0.0) );
        image.copyTo( Mat(padded, Rect(0, image.rows, image.cols, image.rows)) );
        
        /* Create a lookup array for our template values based on the given random x and y points */
        int * xs_ptr = xs.ptr<int>(0),
            * ys_ptr = ys.ptr<int>(0);
        
        vector<float> vals_i1( no_of_points );
        for( int i = 0; i < no_of_points; i++ )
            vals_i1[i] = templ.at<float>(ys_ptr[i] - 1, xs_ptr[i] - 1);
        
        
        /* Recenter our indices */
        Mat xs_centered = xs.clone() - (r1x + 1),
            ys_centered = ys.clone() - (r1y + 1);
        
        int * xs_ptr_cent = xs_centered.ptr<int>(0),
            * ys_ptr_cent = ys_centered.ptr<int>(0);
        
        vector<double> distances(no_of_configs, 0.0 );
        
        /* Calculate the score for each configurations on each of our randomly sampled points */
        tbb::parallel_for( 0, no_of_configs, 1, [&](int i) {

            float a11 = affine_matrices[i].at<float>(0, 0),
                  a12 = affine_matrices[i].at<float>(0, 1),
                  a13 = affine_matrices[i].at<float>(0, 2),
                  a21 = affine_matrices[i].at<float>(1, 0),
                  a22 = affine_matrices[i].at<float>(1, 1),
                  a23 = affine_matrices[i].at<float>(1, 2);
            
            double tmp_1 = (r2x + 1) + a13 + 0.5;
            double tmp_2 = (r2y + 1) + a23 + 0.5 + 1 * image.rows;
            double score = 0.0;
            
            if(!photometric_invariance) {
                for( int j = 0; j < no_of_points; j++ ) {
                    int target_x = int( a11 * xs_ptr_cent[j] + a12 * ys_ptr_cent[j] + tmp_1 ),
                        target_y = int( a21 * xs_ptr_cent[j] + a22 * ys_ptr_cent[j] + tmp_2 );
                    
                    score += abs(vals_i1[j] - padded.at<float>(target_y - 1, target_x - 1) );
                }
            }
            else {
                vector<double> xs_target(no_of_points),
                               ys_target(no_of_points);
                
                double  sum_x         = 0.0,
                        sum_y         = 0.0,
                        sum_x_squared = 0.0,
                        sum_y_squared = 0.0;
                
                for( int j = 0; j < no_of_points; j++ ) {
                    int target_x = int( a11 * xs_ptr_cent[j] + a12 * ys_ptr_cent[j] + tmp_1 ),
                        target_y = int( a21 * xs_ptr_cent[j] + a22 * ys_ptr_cent[j] + tmp_2 );
                    
                    float xi = vals_i1[j],
                          yi = padded.at<float>(target_y - 1, target_x - 1);
                    
                    xs_target[j] = xi;
                    ys_target[j] = yi;
                    
                    sum_x += xi;
                    sum_y += yi;
                    
                    sum_x_squared += (xi * xi);
                    sum_y_squared += (yi * yi);
                }
                
                double  epsilon = 1e-7;
                double  mean_x = sum_x / no_of_points,
                        mean_y = sum_y / no_of_points,
                        sigma_x = sqrt((sum_x_squared - ( sum_x * sum_x ) / no_of_points ) / no_of_points) + epsilon,
                        sigma_y = sqrt((sum_y_squared - ( sum_y * sum_y ) / no_of_points ) / no_of_points) + epsilon;

                double sigma_div = sigma_x / sigma_y;
                double temp = -mean_x + sigma_div * mean_y;
                
                
                for( int j = 0; j < no_of_points; j++ )
                    score += fabs( xs_target[j] - sigma_div * ys_target[j] + temp );
            }
            
            distances[i] = score / no_of_points;
        });
        
        return distances;
    }

    /**
     * Get threshold based on the given delta
     * the hardcoded values are claimed to be experimentally drawn
     */
    float FAsTMatch::getThresholdPerDelta( float delta ) {
        static const float p[2] = {0.1341, 0.0278};
        static const float safety = 0.02;
        
        return p[0] * delta + p[1] - safety;
    }

    /**
     * Given the previously calcuated distances for each configurations,
     * filter out all distances that fall within a certain threshold
     */
    vector<MatchConfig> FAsTMatch::getGoodConfigsByDistance( vector<MatchConfig>& configs, float best_dist, float new_delta,
                                                             vector<double>& distances, float& thresh, bool& too_high_percentage )
    {
        thresh = best_dist + getThresholdPerDelta( new_delta );
        
        /* Only those configs that have distances below the given threshold are */
        /* categorized as good configurations */
        vector<MatchConfig> good_configs;
        for(int i = 0; i < distances.size(); i++ ) {
            if( distances[i] <= thresh )
                good_configs.push_back( configs[i] );
        }
        
        int no_of_configs = static_cast<int>(good_configs.size());
        
        /* Well if there's still too many configurations */
        /* keep shrinking the threshold */
        while( no_of_configs > 27000 ) {
            thresh *= 0.99;
            good_configs.clear();
            
            for(int i = 0; i < distances.size(); i++ ) {
                if( distances[i] <= thresh )
                    good_configs.push_back( configs[i] );
            }
            
            no_of_configs = static_cast<int>(good_configs.size());
        }
        
        assert( no_of_configs > 0 );
        
        float percentage = 1.0 * no_of_configs / configs.size();
        
        /* If it's above 97.8% it's too high percentage */
        too_high_percentage = percentage > 0.022;
        
        return good_configs;
    }


    /**
     * Preprocess image, by first converting it to grayscale
     * then normalizing the value within 0.0 - 1.0 range
     * and finally make sure that the dimensions are in odd values
     **/
    Mat FAsTMatch::preprocessImage( Mat& image ) {
        Mat temp = image.clone();
        if( temp.channels() != 1 )
            cvtColor( temp, temp, CV_BGR2GRAY );
        
        if( temp.type() != CV_32FC1 )
            temp.convertTo( temp, CV_32FC1, 1.0 / 255.0 );
        
        return makeOdd( temp );
    }


    /**
     * If the image dimension is of odd value, leave it as is
     * if it's even, then minus 1 from the dimension
     */
    Mat FAsTMatch::makeOdd(Mat& image) {
        int rows = (image.rows % 2 == 0) ? image.rows - 1 : image.rows;
        int cols = (image.cols % 2 == 0) ? image.cols - 1 : image.cols;
        return Mat( image, Rect(0, 0, cols, rows)).clone();
    }

    /**
     * From the given affine matrix, calculate the four corners of the affine transformed
     * rectangle
     */
    vector<Point2f> FAsTMatch::calcCorners( Size image_size, Size templ_size, Mat& affine ) {
        float   r1x = 0.5 * (templ_size.width - 1),
                r1y = 0.5 * (templ_size.height - 1),
                r2x = 0.5 * (image_size.width - 1),
                r2y = 0.5 * (image_size.height - 1);
        
        float   a11 = affine.at<float>(0, 0),
                a12 = affine.at<float>(0, 1),
                a13 = affine.at<float>(0, 2),
                a21 = affine.at<float>(1, 0),
                a22 = affine.at<float>(1, 1),
                a23 = affine.at<float>(1, 2);
        
        float   templ_w = templ_size.width,
                templ_h = templ_size.height;
        
        /* The four corners of affine transformed template */
        double c1x = a11 * (1-(r1x+1))       + a12*(1-(r1y+1))        + (r2x+1) + a13;
        double c1y = a21 * (1-(r1x+1))       + a22*(1-(r1y+1))        + (r2y+1) + a23;
        
        double c2x = a11 * (templ_w-(r1x+1)) + a12*(1-(r1y+1))        + (r2x+1) + a13;
        double c2y = a21 * (templ_w-(r1x+1)) + a22*(1-(r1y+1))        + (r2y+1) + a23;
        
        double c3x = a11 * (templ_w-(r1x+1)) + a12*(templ_h-(r1y+1))  + (r2x+1) + a13;
        double c3y = a21 * (templ_w-(r1x+1)) + a22*(templ_h-(r1y+1))  + (r2y+1) + a23;
        
        double c4x = a11 * (1-(r1x+1))       + a12*(templ_h-(r1y+1))  + (r2x+1) + a13;
        double c4y = a21 * (1-(r1x+1))       + a22*(templ_h-(r1y+1))  + (r2y+1) + a23;
        
        return vector<Point2f> {
            Point2f(c1x, c1y),
            Point2f(c2x, c2y),
            Point2f(c3x, c3y),
            Point2f(c4x, c4y),
        };
    }
}