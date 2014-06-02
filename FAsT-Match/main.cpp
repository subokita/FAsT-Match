//
//  main.cpp
//  FAsT-Match
//
//  Created by Saburo Okita on 22/05/14.
//  Copyright (c) 2014 Saburo Okita. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>

#include "FAsTMatch.h"
#include "MatchConfig.h"

using namespace std;
using namespace cv;

int main(int argc, const char * argv[])
{

    Mat image = imread( "/Users/saburookita/Personal Projects/FAsT-Match/image.png" );
    Mat templ = imread( "/Users/saburookita/Personal Projects/FAsT-Match/template.png" );
    
    fast_match::FAsTMatch fast_match;
    fast_match.init( 0.15f, 0.85f, false, 0.5f, 2.0f );
    vector<Point2f> corners = fast_match.apply( image, templ );

    
    namedWindow("");
    moveWindow("", 0, 0);
    
    line( image, corners[0], corners[1], Scalar(0, 0, 255), 2);
    line( image, corners[1], corners[2], Scalar(0, 0, 255), 2);
    line( image, corners[2], corners[3], Scalar(0, 0, 255), 2);
    line( image, corners[3], corners[0], Scalar(0, 0, 255), 2);
    
    Mat appended( image.rows, 2 * image.cols, CV_8UC3, Scalar(0, 0, 0) );
    
    CvFont font = cvFontQt("Helvetica", 14.0, CV_RGB(0, 255, 0) );
    addText( appended, "Template: ", Point( 50, 50 ), font );
    templ.copyTo( Mat(appended, Rect((image.cols - templ.cols) / 2, (image.rows - templ.cols) / 2, templ.cols, templ.rows)) );
    image.copyTo( Mat(appended, Rect( image.cols, 0, image.cols, image.rows)) );
    
    imshow("", appended );
    waitKey(0);
    
    return 0;
}

