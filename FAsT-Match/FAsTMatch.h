//
//  FAsTMatch.h
//  FAsT-Match
//
//  Created by Saburo Okita on 23/05/14.
//  Copyright (c) 2014 Saburo Okita. All rights reserved.
//

#ifndef __FAsT_Match__FAsTMatch__
#define __FAsT_Match__FAsTMatch__

#include <iostream>
#include <opencv2/opencv.hpp>

#include "MatchNet.h"
#include "MatchConfig.h"

using namespace std;
using namespace cv;

namespace fast_match {
    class FAsTMatch{
    public:
        FAsTMatch();
        
        void init( float epsilon = 0.15f, float delta = 0.25f, bool photometric_invariance = false,
                   float min_scale = 0.5f, float max_scale = 2.0f );
        
        vector<Point2f> apply(Mat& image, Mat& templ);
        
    protected:
        Mat image, templ;
        
        RNG rng;
        float epsilon;
        float delta;
        bool photometricInvariance;
        float minScale;
        float maxScale;
        Size halfTempl;
        Size halfImage;

        
        
        vector<MatchConfig> createListOfConfigs( MatchNet& net, Size templ_size, Size image_size );
        vector<Mat> configsToAffine( vector<MatchConfig>& configs, vector<bool>& insiders );
        
        vector<MatchConfig> getGoodConfigsByDistance( vector<MatchConfig>& configs, float best_dist, float new_delta,
                                                      vector<double>& distances, float& thresh, bool& too_high_percentage );
        
        vector<MatchConfig> randomExpandConfigs( vector<MatchConfig>& configs, MatchNet& net,
                                                 int level, int no_of_points, float delta_factor );
        
        float getThresholdPerDelta( float delta );
        
        vector<double> evaluateConfigs( Mat& image, Mat& templ, vector<Mat>& affine_matrices,
                                        Mat& xs, Mat& ys, bool photometric_invariance );
        
        
        Mat preprocessImage(Mat& image);
        Mat makeOdd(Mat& image);
        vector<Point2f> calcCorners( Size image_size, Size templ_size, Mat& affine );
        
    };
}

template<typename type>
ostream &operator <<( ostream& os, const std::pair<type, type> & vec ) {
    os << "[";
    os << vec.first << " " << vec.second;
    os << "]";
    return os;
}

template<typename type>
ostream &operator <<( ostream& os, const vector<type> & vec ) {
    os << "[";
    std::copy( vec.begin(), vec.end(), ostream_iterator<type>(os, ", ") );
    os << "]";
    return os;
}

#endif /* defined(__FAsT_Match__FAsTMatch__) */
