//
//  MatchConfig.cpp
//  FAsT-Match
//
//  Created by Saburo Okita on 02/06/14.
//  Copyright (c) 2014 Saburo Okita. All rights reserved.
//

#include "MatchConfig.h"
#include <iomanip>
#include <tbb/tbb.h>

using namespace std;
using namespace cv;

namespace fast_match {
    /**
     * Default constructor
     */
    MatchConfig::MatchConfig(){
    }
    
    /**
     * Constructor
     */
    MatchConfig::MatchConfig( float trans_x, float trans_y, float rotate_2, float scale_x, float scale_y, float rotate_1 ) {
        init( trans_x, trans_y, rotate_2, scale_x, scale_y, rotate_1 );
    }
    
    /**
     * Copy constructor
     */
    MatchConfig::MatchConfig( const MatchConfig& object ) {
        init( object.translateX, object.translateY, object.rotate2, object.scaleX, object.scaleY, object.rotate1 );
    }
    
    /**
     * Initialize our configuration parameters
     */
    void MatchConfig::init( float trans_x, float trans_y, float rotate_2, float scale_x, float scale_y, float rotate_1 ) {
        this->translateX    = trans_x;
        this->translateY    = trans_y;
        this->rotate2       = rotate_2;
        this->scaleX        = scale_x;
        this->scaleY        = scale_y;
        this->rotate1       = rotate_1;
        this->affine        = asAffineMatrix();
    }

    vector<MatchConfig> MatchConfig::fromMatrix(Mat& configs) {
        vector<MatchConfig> result( configs.rows );
        tbb::parallel_for(0, configs.rows, 1, [&]( int i ){
            float * ptr = configs.ptr<float>(i);
            result[i].init( ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5] );
        });
        return result;
    }
    
    /**
     * Create an affine transformation matrix from the configurations
     */
    Mat MatchConfig::asMatrix() {
        return (Mat_<float>(1, 6) << translateX, translateY, rotate2, scaleX, scaleY, rotate1 );
    }
    
    /**
     * Returns the affine matrix representation of the configuration
     */
    Mat MatchConfig::getAffineMatrix() {
        return this->affine;
    }
    
    /**
     * Create an affine transformation matrix from the configurations
     */
    Mat MatchConfig::asAffineMatrix() {
        /* [TODO] Should use lookup table or something in the future */
        float   cos_r1 = cosf( rotate1 ),
                sin_r1 = sinf( rotate1 ),
                cos_r2 = cosf( rotate2 ),
                sin_r2 = sinf( rotate2 );
        
        /* Create affine matrix based on the configuration */
        float   a11 =  scaleX * cos_r1 * cos_r2 - scaleY * sin_r1 * sin_r2,
                a12 = -scaleX * cos_r1 * sin_r2 - scaleY * cos_r2 * sin_r1,
                a21 =  scaleX * cos_r2 * sin_r1 + scaleY * cos_r1 * sin_r2,
                a22 =  scaleY * cos_r1 * cos_r2 - scaleX * sin_r1 * sin_r2;
        
        return (Mat_<float>(2, 3) <<
                a11, a12, translateX,
                a21, a22, translateY );
    }


    /**
     * Stream operator overloading
     */
    ostream& operator<<( ostream& os, const MatchConfig& config ) {
        os << "[" << config.translateX  << ", " << config.translateY << ", "
                  << config.rotate2     << ", " << config.scaleX     << ", "
                  << config.scaleY      << ", " << config.rotate1    << "]";
        return os;
    }
}