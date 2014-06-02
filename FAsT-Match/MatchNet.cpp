//
//  MatchNet.cpp
//  FAsT-Match
//
//  Created by Saburo Okita on 02/06/14.
//  Copyright (c) 2014 Saburo Okita. All rights reserved.
//

#include "MatchNet.h"
#include <cmath>
using namespace std;

namespace fast_match {
    
    MatchNet::MatchNet( const MatchNet& other ) {
        this->boundsTransX = other.boundsTransX;
        this->boundsTransY = other.boundsTransY;
        this->boundsRotate = other.boundsRotate;
        this->boundsScale  = other.boundsScale;
        
        this->stepsTransX = other.stepsTransX;
        this->stepsTransY = other.stepsTransY;
        this->stepsRotate = other.stepsRotate;
        this->stepsScale  = other.stepsScale;
    }
    
    MatchNet::MatchNet( int width, int height, float delta,
                        float min_tx, float max_tx, float min_ty, float max_ty,
                        float min_r, float max_r, float min_s, float max_s )
    {
        this->boundsTransX = { min_tx, max_tx };
        this->boundsTransY = { min_ty, max_ty };
        this->boundsRotate = { min_r, max_r };
        this->boundsScale  = { min_s, max_s };

        this->stepsTransX = delta * width  / sqrt(2.0f);
        this->stepsTransY = delta * height / sqrt(2.0f);
        this->stepsRotate = delta * sqrt( 2.0f );
        this->stepsScale  = delta / sqrt( 2.0f );
    }
    
    
    MatchNet MatchNet::operator*(const float& factor) const {
        MatchNet result(*this);
        result.stepsTransX *= factor;
        result.stepsTransY *= factor;
        result.stepsRotate *= factor;
        result.stepsScale  *= factor;
        return result;
    }
    
    MatchNet MatchNet::operator/(const float& factor) const {
        MatchNet result(*this);
        result.stepsTransX /= factor;
        result.stepsTransY /= factor;
        result.stepsRotate /= factor;
        result.stepsScale  /= factor;
        return result;
    }
    
    
    vector<float> MatchNet::getXTranslationSteps() {
        vector<float> tx_steps;
        for( float x = boundsTransX.first; x <= boundsTransX.second; x += stepsTransX )
            tx_steps.push_back( x );
        
        if( boundsTransX.second - tx_steps[tx_steps.size() - 1] > 0.5 * stepsTransX )
            tx_steps.push_back( tx_steps[tx_steps.size() - 1] + stepsTransX );
        
        return tx_steps;
    }
    
    vector<float> MatchNet::getYTranslationSteps() {
        vector<float> ty_steps;
        for( float y = boundsTransY.first; y <= boundsTransY.second; y += stepsTransY )
            ty_steps.push_back( y );
        
        if( boundsTransY.second - ty_steps[ty_steps.size() - 1] > 0.5 * stepsTransY )
            ty_steps.push_back( ty_steps[ty_steps.size() - 1] + stepsTransY );
        
        return ty_steps;
    }
    
    vector<float> MatchNet::getRotationSteps() {
        vector<float> r_steps;
        for( float r = boundsRotate.first; r <= boundsRotate.second; r += stepsRotate )
            r_steps.push_back( r );
        
        if( boundsRotate.second - r_steps[r_steps.size() - 1] > stepsRotate )
            r_steps.push_back( r_steps[r_steps.size() - 1] + stepsRotate );
        
        return r_steps;
    }
    
    vector<float> MatchNet::getScaleSteps() {
        vector<float> s_steps;
        for( float s = boundsScale.first; s <= boundsScale.second; s += stepsScale )
            s_steps.push_back( s );
        
        if( stepsScale == 0.0 )
            s_steps = { boundsScale.first };
        
        if( boundsScale.second - s_steps[s_steps.size() - 1] > 0.5 * stepsScale )
            s_steps.push_back( s_steps[s_steps.size() - 1] + stepsScale );
        
        return s_steps;
    }
}