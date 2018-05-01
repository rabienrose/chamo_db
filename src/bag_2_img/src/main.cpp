#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char **argv){
    std::string bag_addr=argv[1];
    std::string bag_name=argv[2];
    rosbag::Bag bag;
    bag.open(bag_addr+"/"+bag_name+".bag",rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("/cam0/image_raw");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
        if(simg!=NULL){
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                simg->height=480;
                simg->width=640;
                simg->step=simg->width;
                simg->is_bigendian=true;
                simg->encoding="mono8";
                cv_ptr = cv_bridge::toCvCopy(simg, "mono8");
                //cv::Mat img_c;
                //cv::cvtColor(cv_ptr->image, img_c, CV_GRAY2BGR);
                std::stringstream ss;
                ss<<"./img/img_"<<img_count<<".jpg";
                //cv::imshow("chamo",cv_ptr->image);
                //cv::waitKey(-1);
                cv::imwrite(ss.str(), cv_ptr->image);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return 0;
            }
            img_count++;
        }
    }
    
    return 0;
};
