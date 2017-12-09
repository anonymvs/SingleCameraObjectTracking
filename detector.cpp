//
// Created by hegedus on 2017.11.26..
//

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <opencv2/core/ocl.hpp>

#include "detector.h"

using namespace cv;

//gnuplottal 3d vizualizáció

Detection::Detection() : lowTresh{HSV_RED_LOWTRESHOLD}, highTresh{HSV_RED_HIGHTRESHOLD}, type{Type::inRange_HSV} {}

Detection::Detection(Scalar lowTresh = Scalar(0,0,0),
                     Scalar highTresh = Scalar(255,255,255),
                     Type type = Type::inRange_HSV) : lowTresh{lowTresh}, highTresh{highTresh}, type{type} {}

// changes the Mat parameter into a masked blurred HSV
void Detection::inRangeDetectionHSV(Mat &frame) {
    Mat hsv;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    
    /*if(lowTresh == Scalar(0,0,0) && highTresh == Scalar(255,255,255) && type == Type::inRange_HSV) {
        lowTresh = HSV_RED_LOWTRESHOLD;
        highTresh = HSV_RED_HIGHTRESHOLD;
    }*/

    lowTresh = HSV_HD_VID_RED_LOWTRESHOLD;
    highTresh = HSV_HD_VID_RED_HIGHTRESHOLD;

    inRange(hsv, lowTresh, highTresh, frame);
    GaussianBlur(frame, frame, Size(15, 15), 0, 0);
/*    namedWindow("mask", WINDOW_AUTOSIZE);
    imshow("mask", frame);*/
}

void Detection::inRangeDetectionYUV(Mat &frame) {
    Mat rem_spek;
    cvtColor(frame, rem_spek, CV_BGR2YUV);
    std::vector<Mat> channels;
    split(rem_spek, channels);
    equalizeHist(channels[1], channels[1]);
    merge(channels, rem_spek);

    if(lowTresh == Scalar(0,0,0) && highTresh == Scalar(255,255,255) && type == Type::inRange_YUY) {
        lowTresh = HSV_RED_LOWTRESHOLD;
        highTresh = HSV_RED_HIGHTRESHOLD;
    }

    inRange(rem_spek, lowTresh, highTresh, frame);
    GaussianBlur(frame, frame, Size(9, 9), 2, 2);
}


void Detection::detect(Mat frame, std::vector<Vec3f> &circles) {
    inRangeDetectionHSV(frame);
    //HoughCircles(frame, circles, CV_HOUGH_GRADIENT, 1, 50, 70, 35, 0, 0);
    HoughCircles(frame, circles, CV_HOUGH_GRADIENT, 0.5, frame.rows / 8, 40, 35, 0, 0);
}

void Detection::drawCircles(Mat &frame, std::vector<Vec3f> circles, Scalar color) {
    if(circles.empty()) return;
    for(size_t i = 0; i < circles.size(); ++i) {
        cv::Point center(std::round(circles[i][0]), std::round(circles[i][1]));
        int radius = std::round(circles[i][2]);

        cv::circle(frame, center, radius, color, 5);
    }
}

cv::Mat Detection::getMask(cv::Mat src) {
    inRangeDetectionHSV(src);
    return src;
}

