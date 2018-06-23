#include <iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>

using namespace std;

cv::Mat colorDetection(cv::Scalar colorLow, cv::Scalar colorHigh, cv::Scalar color, cv::Mat frame) {
//    hsvColorCode
//        blue: 255,10,10
//        yello: 0,255,255
//        green: 0,255,10
//        white: 255,255,255
//        red: 70,10,120
//        orange:30,80,200

    cv::Mat hsvFrame, inRangeFrame;
    cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);
    cv::inRange(hsvFrame, colorLow, colorHigh, inRangeFrame);
    cv::Mat erosionElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15,15));
//    cv::namedWindow("before", cv::WINDOW_NORMAL);
//    cv::resizeWindow("before", cv::Size(500,500));
//    cv::namedWindow("after",  cv::WINDOW_NORMAL);
//    cv::resizeWindow("after", cv::Size(500,500));
//    cv::imshow("before", inRangeFrame);
    cv::erode(inRangeFrame, inRangeFrame, erosionElement);
    cv::dilate(inRangeFrame, inRangeFrame, dilateElement);
//    cv::imshow("after", inRangeFrame);


    //find contours and trace lines
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(inRangeFrame, contours, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (int i = 0;i < contours.size();i++) {
        cv::drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, cv::Point());

    }

    return frame;
}

bool colorTrack() {
    const char* path = "autovideosrc device=/dev/video0  ! appsink";
    cv::VideoCapture webcam(path);
    if (!webcam.isOpened()) {
        cout << "Webcam not connected" << endl;
        return false;
    }

    int blueLow_H = 91, blueLow_S = 121, blueLow_V = 47;
    int blueHigh_H = 130, blueHigh_S = 255, blueHigh_V = 255;
    int greenLow_H = 46, greenLow_S = 143, greenLow_V = 41;
    int greenHigh_H = 84, greenHigh_S = 255, greenHigh_V = 145;
    int yellowLow_H = 29, yellowLow_S = 73, yellowLow_V = 87;
    int yellowHigh_H = 65, yellowHigh_S = 234, yellowHigh_V = 208;
        int orangeLow_H = 0, orangeLow_S = 157, orangeLow_V = 117;
        int orangeHigh_H = 30, orangeHigh_S = 255, orangeHigh_V = 198;
        int redLow_H = 0, redLow_S = 202, redLow_V = 9;
        int redHigh_H = 7, redHigh_S = 244, redHigh_V = 116;
        int whiteLow_H = 19, whiteLow_S = 0, whiteLow_V = 110;
        int whiteHigh_H = 72, whiteHigh_S = 44, whiteHigh_V = 219;

    cv::Size windowSize(500,500);

    cv::namedWindow("objectDetect", cv::WINDOW_NORMAL);
    cv::Mat frame;
    for (;;) {
        webcam >> frame;
        if (frame.empty()) {
            cout << "Frame is empty" << endl;
            return false;
        }
        frame = colorDetection(cv::Scalar(blueLow_H, blueLow_S, blueLow_V), cv::Scalar(blueHigh_H, blueHigh_S, blueHigh_V), cv::Scalar(255,10,10), frame);
        frame = colorDetection(cv::Scalar(greenLow_H, greenLow_S, greenLow_V), cv::Scalar(greenHigh_H, greenHigh_S, greenHigh_V), cv::Scalar( 0,255,10), frame);
        frame = colorDetection(cv::Scalar(yellowLow_H, yellowLow_S, yellowLow_V), cv::Scalar(yellowHigh_H, yellowHigh_S, yellowHigh_V), cv::Scalar( 0,255,255), frame);
            frame = colorDetection(cv::Scalar(orangeLow_H, orangeLow_S, orangeLow_V), cv::Scalar(orangeHigh_H, orangeHigh_S, orangeHigh_V), cv::Scalar(30,80,200), frame);
//            frame = colorDetection(cv::Scalar(redLow_H, redLow_S, redLow_V), cv::Scalar(redHigh_H, redHigh_S, redHigh_V), cv::Scalar(70,10,120), frame);
//            frame = colorDetection(cv::Scalar(whiteLow_H, whiteLow_S, whiteLow_V), cv::Scalar(whiteHigh_H, whiteHigh_S, whiteHigh_V), cv::Scalar(255,255,255), frame);


        cv::imshow("objectDetect", frame);
        if (cv::waitKey(33) > 0) return false;
    }

//    int blueLow_H = 91, blueLow_S = 121, blueLow_V = 47;
//    int blueHigh_H = 130, blueHigh_S = 255, blueHigh_V = 255;

//    cv::Size windowSize(500,500);
//    cv::namedWindow("Original", cv::WINDOW_NORMAL);
//    cv::resizeWindow("Original",windowSize);

//    /*create bars for configuring best HSV range*/
//    cv::createTrackbar("lowH", "Original", &blueLow_H, 255);
//    cv::createTrackbar("lowS", "Original", &blueLow_S, 255);
//    cv::createTrackbar("lowV", "Original", &blueLow_V, 255);
//    cv::createTrackbar("highH", "Original", &blueHigh_H, 255);
//    cv::createTrackbar("highS", "Original", &blueHigh_S, 255);
//    cv::createTrackbar("highV", "Original", &blueHigh_V, 255);

//    cv::namedWindow("HSV", cv::WINDOW_NORMAL);
//    cv::resizeWindow("HSV",windowSize);

//    cv::namedWindow("inRange", cv::WINDOW_NORMAL);
//    cv::resizeWindow("inRange",windowSize);

//    cv::Mat frame, hsvFrame, inRange, objectDetection;
//    vector<vector<cv::Point>> contours;
//    vector<cv::Vec4i> hierarchy;

//    for (;;) {
//        webcam >> frame;
//        if (frame.empty()) {
//            cout << "Frame is empty" << endl;
//            return false;
//        }
//        cv::imshow("Original", frame);

//        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV); //convert to HSV
//        cv::imshow("HSV", hsvFrame);

//        cv::inRange(hsvFrame, cv::Scalar(blueLow_H, blueLow_S, blueLow_V), cv::Scalar(blueHigh_H, blueHigh_S, blueHigh_V), inRange); //filter out everything except in range 'blue'
//        cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//        morphologyEx(inRange, inRange, cv::MORPH_OPEN, str_el);
//        morphologyEx(inRange, inRange, cv::MORPH_CLOSE, str_el);

//        cv::imshow("inRange", inRange);
//        cv::findContours(inRange, contours, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);
//        for (int i = 0;i < contours.size();i++) {
//            cv::drawContours(frame, contours, i, cv::Scalar(0,0,0), 2, 8, hierarchy, 0, cv::Point());
//        }

//        cv::imshow("objectDetect", frame);
//        if (cv::waitKey(33) > 0) return false;
//    }
}

int main(int argc, char *argv[])
{
    colorTrack();

    return 0;
}

