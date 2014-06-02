//
//  MatchNet.h
//  FAsT-Match
//
//  Created by Saburo Okita on 02/06/14.
//  Copyright (c) 2014 Saburo Okita. All rights reserved.
//

#ifndef __FAsT_Match__MatchNet__
#define __FAsT_Match__MatchNet__

#include <iostream>
#include <vector>

namespace fast_match {
    
    /**
     * A representation of the 
     */
    class MatchNet {
    public:
        MatchNet( int width, int height, float delta,
                  float min_tx, float max_tx, float min_ty, float max_ty,
                  float min_r, float max_r, float min_s, float max_s );
        
        MatchNet( const MatchNet& other );
        
        
        MatchNet operator*(const float& factor)const;
        MatchNet operator/(const float& factor) const;
        
        std::vector<float> getXTranslationSteps();
        std::vector<float> getYTranslationSteps();
        std::vector<float> getRotationSteps();
        std::vector<float> getScaleSteps();
        
        std::pair<float, float> boundsTransX;
        std::pair<float, float> boundsTransY;
        std::pair<float, float> boundsRotate;
        std::pair<float, float> boundsScale;
        
        float stepsTransX;
        float stepsTransY;
        float stepsRotate;
        float stepsScale;
        
        
        
    };
}
#endif /* defined(__FAsT_Match__MatchNet__) */
