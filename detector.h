//
// Created by hegedus on 2017.11.26..
//

#ifndef SINGLECAMERAOBJECTTRACKING_DETECTION_H
#define SINGLECAMERAOBJECTTRACKING_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <opencv2/core/ocl.hpp>

enum Type {
    inRange_HSV,
    inRange_YUY
};

static const cv::Scalar HSV_RED_LOWTRESHOLD = cv::Scalar(150, 140, 140);
static const cv::Scalar HSV_RED_HIGHTRESHOLD = cv::Scalar(180, 255, 255);

static const cv::Scalar HSV_HD_RED_LOWTRESHOLD = cv::Scalar(3, 0, 0);
static const cv::Scalar HSV_HD_RED_HIGHTRESHOLD = cv::Scalar(13, 240, 255);

static const cv::Scalar HSV_HD_VID_RED_LOWTRESHOLD = cv::Scalar(0,88, 109);
static const cv::Scalar HSV_HD_VID_RED_HIGHTRESHOLD = cv::Scalar(13,255, 255);

static const cv::Scalar YUV_RED_LOWTRESHOLD = cv::Scalar(80, 20, 150);
static const cv::Scalar YUV_RED_HIGHTRESHOLD = cv::Scalar(230, 170, 180);

class Detection {
private:
    cv::Scalar lowTresh;
    cv::Scalar highTresh;
    Type type;

    void inRangeDetectionHSV(cv::Mat &);
    void inRangeDetectionYUV(cv::Mat &);
    void cannyEdgeDetection(cv::Mat &);

public:
    Detection();

    Detection(cv::Scalar, cv::Scalar, Type);

    void detect(cv::Mat, std::vector<cv::Vec3f> &);

    void drawCircles(cv::Mat &, std::vector<cv::Vec3f>, cv::Scalar);

    cv::Mat getMask(cv::Mat);

    cv::Mat getCannyEdge(cv::Mat);

    float distanceToCamera(float, float, float);

    float calculateFOV(float, int);
};


#endif //SINGLECAMERAOBJECTTRACKING_DETECTION_H
