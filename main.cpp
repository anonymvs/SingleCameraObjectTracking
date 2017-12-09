//
// Created by hegedus on 2017.11.26..
//

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <opencv2/core/ocl.hpp>

#include <iostream>

#include "detector.h"

using namespace std;
using namespace cv;

Mat generateOutput(Mat im1, Mat im2) {
    Size sz1 = im1.size();
    Size sz2 = im2.size();
    Mat ret(sz1.height, sz1.width + sz2.width, CV_8UC3);
/*
    ret.adjustROI(0, 0, 0, -sz2.width);
    im1.copyTo(ret);

    ret.adjustROI(0, 0, -sz1.width, sz2.width);
    im2.copyTo(ret);

    ret.adjustROI(0, 0, sz1.width, 0);*/

    Mat left(ret, Rect(0, 0, sz1.width, sz1.height));
    im1.copyTo(left);

    Mat right(ret, Rect(sz1.width, 0, sz2.width, sz2.height));
    im2.copyTo(right);
    return ret;
}


int main(int argc, char** argv) {
    if(argc != 2) {
        cout << "Missing parameter";
        return -1;
    }

    VideoCapture video(argv[1]);

    if(!video.isOpened()) {
        cout << "Could not read video file\n";
        return 1;
    }

    Mat frame;
    vector<Vec3f> circles;
    Detection detection;
    video.read(frame);

    Mat mask;

    bool state = true;
    bool cont = true;
    while (state) {

        int key = waitKey(30);
        switch(key) {
            case (27):
                state = false;
            case (97):
                cont = true;
                break;
            case (98):
                cont = false;
                break;
            default:
                break;
        }

        resize(frame, frame, Size(1024, 576), 0, 0, INTER_LANCZOS4);
        //resize(frame, frame, Size(640, 360), 0, 0, INTER_LANCZOS4);
        detection.detect(frame, circles);
        detection.drawCircles(frame, circles, Scalar(255,0,0));
        mask = detection.getMask(frame);
        detection.drawCircles(mask, circles, Scalar(255,0,0));


        //Mat output = generateOutput(frame, mask);

        namedWindow("default window", WINDOW_AUTOSIZE);
        imshow("default window", frame);
        imshow("not def window", mask);

        if(cont) video.read(frame);
    }

    return 0;
}
