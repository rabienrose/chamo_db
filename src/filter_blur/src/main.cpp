#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>

int main(int argc, char **argv){
    std::string img_addr=argv[1];
    cv::Mat raw_img = cv::imread(img_addr.c_str());
    cv::Mat gray_img;
    cv::cvtColor(raw_img, gray_img , CV_BGR2GRAY);
    cv::Mat lap_img;
    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    cv::Laplacian( gray_img, lap_img, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT );
    cv::Mat mean;
    cv::Mat stddev;
    cv::meanStdDev(lap_img, mean, stddev);
    std::cout<<(int)stddev.at<double>(0,0)<<std::endl;
    //cv::Mat abs_lap_img;
    //cv::convertScaleAbs( lap_img, abs_lap_img );
    //cv::imshow("chamo", gray_img);
    //cv::imshow("lap", abs_lap_img);
    //cv::waitKey(-1);
    return 0;
};
