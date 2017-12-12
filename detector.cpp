//
// Created by hegedus on 2017.11.26..
//

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <opencv2/core/ocl.hpp>

#include "detector.h"

using namespace cv;

Detection::Detection() : lowTresh{HSV_RED_LOWTRESHOLD}, highTresh{HSV_RED_HIGHTRESHOLD}, type{Type::inRange_HSV} {}

Detection::Detection(Scalar lowTresh = Scalar(0,0,0),
                     Scalar highTresh = Scalar(255,255,255),
                     Type type = Type::inRange_HSV) : lowTresh{lowTresh}, highTresh{highTresh}, type{type} {}

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
    Mat mask;
    resize(frame, mask, Size(1024, 576), 0, 0, INTER_LANCZOS4);
    imshow("bla", mask);
}

void Detection::detect(Mat frame, std::vector<Vec3f> &circles) {
    inRangeDetectionHSV(frame);
    HoughCircles(frame, circles, CV_HOUGH_GRADIENT, 0.5, frame.rows / 8, 40, 35, 0, 0);
    std::cout << circles.size() << "\n";
}

void Detection::drawCircles(Mat &frame, std::vector<Vec3f> circles, Scalar color) {
    if(circles.empty()) return;
    for(size_t i = 0; i < circles.size(); ++i) {
        cv::Point2f center(std::round(circles[i][0]), std::round(circles[i][1]));
        int radius = (int) std::round(circles[i][2]);

        cv::circle(frame, center, radius, color, 10);
    }
}

cv::Mat Detection::getMask(cv::Mat src) {
    inRangeDetectionHSV(src);
    return src;
}
cv::Mat Detection::getCannyEdge(cv::Mat src) {
    cannyEdgeDetection(src);
    return src;
}

float euclideanDist(Point2f p, Point2f q) {
    Point2f diff = p -q;
    return sqrtf(diff.x * diff.x + diff.y * diff.y);
}
cv::Vec3f avgCircles(cv::Vec3f &lhs, cv::Vec3f &rhs) {
    cv::Vec3f ret(
            (lhs[0] + rhs[0]) / 2,
            (lhs[1] + rhs[1]) / 2,
            (lhs[2] + rhs[2]) / 2
    );
    return ret;
}

float Detection::distanceToCamera(float known_width, float focalLength, float perWidth) {
    /*char buff[50];
    sprintf(buff, "distance = (%f * %f) / %f\n", known_width, focalLength, perWidth);
    std::cout << buff;*/
    return (known_width * focalLength) / perWidth;
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
void Detection::cannyEdgeDetection(Mat &frame) {
    Mat gray;

    int tresh = 100;
    cvtColor(frame, gray, CV_BGR2GRAY);
    Canny(gray, frame, tresh, tresh * 2, 3);
}

float Detection::calculateFOV(float f, int w) {
    float ret = 2 * tan((w / 2) / f);
    return ret;
}

cv::Point3f Detection::calculateCoordinates(float const focalLength, float const distance, int const height, int const width, float const x, float const y) {
    Vec3f camera{0,0,0};
    Vec3f plane_point{
            x - height / 2,
            y - width / 2,
            focalLength
    };
    Vec3f dir = plane_point - camera;
    float d = sqrtf(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
    Vec3f dir_unit{dir[0] / d, dir[1] / d, dir[2] / d};
    Point3f p{
            camera[0] + dir_unit[0] * distance,
            camera[1] + dir_unit[1] * distance,
            camera[2] + dir_unit[2] * distance
    };
    return p;
}



