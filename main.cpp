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

void imageDetection(char* path);

void videoDetection(char* path);

void calculationsOnFrame(Mat &frame, Detection &detection, std::vector<cv::Vec3f> circles, Mat &mask);

void runTest();

void saveTimeline();

void savePoints();


float focalLength = 3272.61360667f;
float HFOV = 1.32963f;  // 76.18°
float VFOV = 0.68511f;  // 39,25°
static float KNOWN_DISTANCE = 30.0f;
static float KNOWN_WIDTH = 7.5f;

std::vector<float> timeline;
std::vector<Point3f> points;

int main(int argc, char** argv) {
    if(argc != 3 && argc != 4) {
        if(argc == 1) {
            runTest();
            return 0;
        }
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
            //-c /home/hegedus/Videos/IMG_20171208_235426.jpg /home/hegedus/Videos/ball_camera_120.mp4
            if(argc == 3) {
                cout << "Missing parameter";
                return -1;
            }
            std::cout << argv[2] << "\n" << argv[3];
            //imageDetection(argv[2]);
            videoDetection(argv[3]);
            break;
        case 'f':
            imageDetection((char*)"/home/hegedus/Videos/red_ball_30cm.jpg");
            KNOWN_DISTANCE = 60.0f;
            imageDetection((char*)"/home/hegedus/Videos/red_ball_60cm.jpg");
            KNOWN_DISTANCE = 90.0f;
            imageDetection((char*)"/home/hegedus/Videos/red_ball_90cm.jpg");
            KNOWN_DISTANCE = 120.0f;
            imageDetection((char*)"/home/hegedus/Videos/red_ball_120cm.jpg");
        default:
            std::cout << "Invalid parameters\n";
    }

    return 0;
}

void runTest() {
    Mat frame, mask;
    std::vector<Vec3f> circles;
    Detection detection;

    int i = 30;
    bool calculated = false;
    bool loop = true;
    while(loop) {

        int key = waitKey(30);
        switch(key) {
            case 'n': {
                if(i < 200) {
                    i += 5;
                    calculated = false;
                } else {
                    saveTimeline();
                    savePoints();
                    return;
                }
                break;
            }
            case (27):
                return;
            default:
                break;
        }

        string path = "/home/hegedus/Videos/test_engineer/Photos/";
        string name = to_string(i) + ".jpg";
        string full_path = path + name;
        frame = imread(full_path, 1);
        if(!frame.data) {
            std::cout << "no image data";
            return;
        }
        if(!calculated) {
            rectangle(frame, Rect(0, 0, 2400, 450), Scalar(255,255,255), CV_FILLED, 8, 0);
            calculationsOnFrame(frame, detection, circles, mask);
            calculated = true;
            double scale = 4;
            Point org_irl = cvPoint(10,100);
            putText(frame, "Distance irl: " + to_string(i) + " cm", org_irl, FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 0, 255, 255), 8);

            resize(frame, frame, Size(1920, 1080), 0, 0, INTER_LANCZOS4);
            imshow("test", frame);
        }

    }

}

void calculationsOnFrame(Mat &frame, Detection &detection, std::vector<cv::Vec3f> circles, Mat &mask) {
    detection.detect(frame, circles);
    detection.drawCircles(frame, circles, Scalar(255, 0, 0));

    /*mask = detection.getMask(frame);
    detection.drawCircles(mask, circles, Scalar(255, 0, 0));*/
    if(!circles.empty() && circles.size() == 1) {
        char fps_str[50];
        char coordinate_str[1000];
        float distance = detection.distanceToCamera(KNOWN_WIDTH, focalLength, circles[0][2] * 2);
        Point3f point = detection.calculateCoordinates(focalLength, distance, frame.rows, frame.cols, circles[0][0], circles[0][1]);
        timeline.push_back(distance);
        points.push_back(point);
        sprintf(fps_str, "Distance: %f cm ", distance);
        sprintf(coordinate_str, "Point( x = %.lf, y = %.lf, z = %.lf ) ", point.x, point.y, point.z);
        double scale = 4;
        //Point org = cvPoint(frame.cols / 7, frame.rows / 14);
        Point org = cvPoint(10, 250);
        //Point org_point = cvPoint(frame.cols / 7, frame.rows / 4);
        Point org_point = cvPoint(10, 400);
        putText(frame, fps_str, org, FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 0, 255, 255), 8);
        putText(frame, coordinate_str, org_point, FONT_HERSHEY_SIMPLEX, scale, Scalar(0, 0, 255, 255), 8);
    }
}

void saveTimeline() {
    /*ofstream file;
    file.open("TIMELINE.txt");
    for(size_t i = 0; i < timeline.size(); i++) {
        file << timeline[i] << "\n";
    }
    file.close();
    std::cout << "file(timeline) has been written";*/
}

void savePoints() {
    /*ofstream file;
    file.open("POINTS.txt");
    for(size_t i = 0; i < points.size(); i++) {
        file << points[i] << "\n";
    }
    file.close();
    std::cout << "file(points) has been written";*/
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

    bool state = true;
    bool cont = true;
    while (state) {

        int key = waitKey(30);
        switch(key) {
            case (27):
                state = false;
            case (97): {
                if(!video.read(frame))
                    state = false;
                else
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
                savePoints();
                break;
            default:
                break;
        }

        resize(frame, frame, Size(1920, 1080), 0, 0, INTER_LANCZOS4);
        //resize(mask, mask, Size(1024, 576), 0, 0, INTER_LANCZOS4);
        imshow("default window", frame);
        //imshow("not def window", mask);

        if(cont) {
            if(!video.read(frame)) {
                state = false;
                saveTimeline();
                savePoints();
            } else {
                calculationsOnFrame(frame, detection, circles, mask);
            }
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

    Mat frame = image;
    std::vector<cv::Vec3f> circles;

    frame = imread(path, 1);
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
    //std::cout << "frame width: " << frame.cols << "\t frame height: " << frame.rows << "\n";
    float HFOV = detection.calculateFOV(focalLength, frame.cols);
    float VFOV = detection.calculateFOV(focalLength, frame.rows);
    Point3f coord = detection.calculateCoordinates(focalLength, distance, frame.cols, frame.rows, circles[0][0], circles[0][1]);
    std::cout << HFOV << "\t" << VFOV << "\n";
    std::cout << "distance = " << distance << "\n";

}





