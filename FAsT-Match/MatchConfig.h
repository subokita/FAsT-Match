//
//  MatchConfig.h
//  FAsT-Match
//
//  Created by Saburo Okita on 02/06/14.
//  Copyright (c) 2014 Saburo Okita. All rights reserved.
//

#ifndef __FAsT_Match__MatchConfig__
#define __FAsT_Match__MatchConfig__

#include <iostream>
#include <opencv2/opencv.hpp>

namespace fast_match {
    /**
     * Config class that describes the parameters used in creating affine transformations
     */
    class MatchConfig {
    public:
        MatchConfig();
        MatchConfig( const MatchConfig& object );
        MatchConfig( float trans_x, float trans_y, float rotate_2, float scale_x, float scale_y, float rotate_1 );
        void init( float trans_x, float trans_y, float rotate_2, float scale_x, float scale_y, float rotate_1 );
        
        static std::vector<MatchConfig> fromMatrix( cv::Mat& configs);
        cv::Mat asMatrix();
        cv::Mat asAffineMatrix();
        cv::Mat getAffineMatrix();
        
        friend std::ostream &operator <<( std::ostream& os, const MatchConfig & conf );
        
        
    protected:
        float translateX;
        float translateY;
        float rotate2;
        float rotate1;
        float scaleX;
        float scaleY;
        
        cv::Mat affine;
    };
}



#endif /* defined(__FAsT_Match__MatchConfig__) */
