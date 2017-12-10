//
// Created by hegedus on 2017.11.26..
//

#include <opencv2/opencv.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/tracking/tldDataset.hpp>
#include <opencv2/core/ocl.hpp>

#include <iostream>

#include "detector.h"

void imageDetection();

void videoDetection();

using namespace std;
using namespace cv;

const char * VIDEO_PATH = "/home/hegedus/Videos/ball_camera_120.mp4";
float focalLength = 3272.61360667;
static float KNOWN_DISTANCE = 30.0f;
static float KNOWN_WIDTH = 7.5f;

std::vector<float> timeline;

//-c /home/hegedus/Videos/IMG_20171208_235426.jpg /home/hegedus/Videos/ball_camera_120.mp4

void calculationsOnFrame(Mat &frame, Detection &detection, std::vector<cv::Vec3f> circles, Mat &mask) {
    detection.detect(frame, circles);
    detection.drawCircles(frame, circles, Scalar(255, 0, 0));

    mask = detection.getMask(frame);
    detection.drawCircles(mask, circles, Scalar(255, 0, 0));
    if(!circles.empty() && circles.size() == 1) {
        char fps_str[50];
        float distance = detection.distanceToCamera(KNOWN_WIDTH, focalLength, circles[0][2] * 2);
        timeline.push_back(distance);
        sprintf(fps_str, "Distance: %f cm", distance);
        double scale = 4;
        Point org = cvPoint(frame.cols / 7, frame.rows / 14);
        putText(frame, fps_str, org, FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 0, 255, 255), 8);
    }
}

void saveTimeline() {
    ofstream file;
    file.open("TIMELINE.txt");
    for(size_t i = 0; i < timeline.size(); i++) {
        file << timeline[i] << "\n";
    }
    file.close();
    std::cout << "file has been written";
}

void videoDetection(char* path) {
    std::cout << "video\n";
    VideoCapture video(path);

    if(!video.isOpened()) {
        cout << "Could not read video file\n";
        return;
    }

    Mat frame;
    vector<Vec3f> circles;
    Detection detection;
    video.read(frame);
    resize(frame, frame, Size(1024, 576), 0, 0, INTER_LANCZOS4);
    imshow("default window", frame);

    Mat mask;
    int cnt = 0;

    bool state = true;
    bool cont = false;
    while (state) {

        int key = waitKey(30);
        switch(key) {
            case (27):
                state = false;
            case (97): {
                video.read(frame);
                calculationsOnFrame(frame, detection, circles, mask);
                break;
            }
            case (98):
                cont = false;
                break;
            case (99):
                cont = true;
                break;
            case (100):
                saveTimeline();
                break;
            default:
                break;
        }

        resize(frame, frame, Size(1024, 576), 0, 0, INTER_LANCZOS4);
        //resize(mask, mask, Size(1024, 576), 0, 0, INTER_LANCZOS4);
        imshow("default window", frame);
        //imshow("not def window", mask);

        if(cont) {
            video.read(frame);
            calculationsOnFrame(frame, detection, circles, mask);
        }
    }
}

void imageDetection(char* path) {
    Mat image;
    Detection detection;
    image = imread(path, 1);
    if (!image.data) {
        std::cout << "No image data \n";
        return;
    }

    namedWindow("def", WINDOW_AUTOSIZE);

    Mat frame = image;
    std::vector<cv::Vec3f> circles;

    frame = imread(path, 1);
    //resize(frame, frame, Size(1024, 576), 0, 0, INTER_LANCZOS4);
    //imshow("def", frame);
    detection.detect(frame, circles);
    detection.drawCircles(frame, circles, Scalar(255, 0, 0));
    imshow("def", frame);

    while (true) {
        if(waitKey(0) == 27) {
            break;
        }
        imshow("def", frame);
    }

    focalLength = (circles[0][2] * 2 * KNOWN_DISTANCE) / KNOWN_WIDTH;
    char buff[50];
    sprintf(buff, "%f = (%f * 2 * %f) / %f\n", focalLength, circles[0][2], KNOWN_DISTANCE, KNOWN_WIDTH);
    std::cout << buff;
    float distance = detection.distanceToCamera(KNOWN_WIDTH, focalLength, circles[0][2] * 2);
    std::cout << "distance = " << distance << "\n";

}


int main(int argc, char** argv) {
    if(argc != 3 && argc != 4) {
        cout << "Missing parameter";
        return -1;
    }

    switch (argv[1][1]) {
        case 'i' :
            imageDetection(argv[2]);
            break;
        case 'v' :
            videoDetection(argv[2]);
            break;
        case 'c' :
            if(argc == 3) {
                cout << "Missing parameter";
                return -1;
            }
            std::cout << argv[2] << "\n" << argv[3];
            //imageDetection(argv[2]);
            /*imageDetection("/home/hegedus/Videos/red_ball_30cm.jpg");
            KNOWN_DISTANCE = 60.0f;
            imageDetection("/home/hegedus/Videos/red_ball_60cm.jpg");
            KNOWN_DISTANCE = 90.0f;
            imageDetection("/home/hegedus/Videos/red_ball_90cm.jpg");
            KNOWN_DISTANCE = 120.0f;
            imageDetection("/home/hegedus/Videos/red_ball_120cm.jpg");*/
            videoDetection(argv[3]);
            break;
        default:
        std::cout << "Invalid parameters\n";
    }

    return 0;
}


