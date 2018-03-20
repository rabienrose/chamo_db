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
#include <save_bag/img_chamo.h>
#include <cloud_expand/connection.h>


class Frame{
public:
    int frame_id;
    cv::Mat img;
    std::vector<cv::Mat> descs;
    std::vector<cv::KeyPoint> kps;
};

class Connection{
public:
    int frame_id1;
    int kp_id1;
    int frame_id2;
    int kp_id2;
};

class SystemFront{
public:
    bool init(){
        
    }
    bool processData(cv::Mat img, std::vector<Connection>& outConn){
        cv::Ptr<cv::ORB> extractor=cv::ORB::create();
        std::vector<cv::KeyPoint> kps;
        cv::Mat mask;
        cv::Mat descs;
        extractor->detectAndCompute(img, mask, kps, descs);
        cv::Mat debug_img= img.clone();
        for (int i=0; i<(int)kps.size();i++){
            cv::circle(debug_img, kps[i].pt, 1, cv::Scalar(255,0,0,255), 2);
        }
        cv::imshow("chamo", debug_img);
        cv::waitKey(-1);
        
    }
private:
    std::list<Frame> frame_list;
};

int main(int argc, char **argv){
    //float fx,fy,cx, cy, k1, k2, k3, p1, p2;
    ros::init(argc, argv, "cloud_expand");
    ros::NodeHandle nn;
    std::string bag_addr="/media/psf/Home/Documents/data/bag4chamodb/2018-03-20-23-43-14.bag";
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    rosbag::Bag bag_out;
    bag_out.open("./output/chamo.bag", rosbag::bagmode::Write);
    SystemFront mySys;
    mySys.init();
    std::vector<std::string> topics;
    topics.push_back("img_chamo");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        save_bag::img_chamoConstPtr simg = m.instantiate<save_bag::img_chamo>();
        if(simg!=NULL){
            img_count++;
            cv::Mat temp_img;
            temp_img = cv::imdecode(simg->jpg, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);
            std::vector<Connection> outConn;
            mySys.processData(temp_img, outConn);
            cloud_expand::connection conn_msg;
            for (int i=0;i<(int)outConn.size();i++){
                conn_msg.frame_id1.push_back(outConn[i].frame_id1);
                conn_msg.frame_id2.push_back(outConn[i].frame_id2);
                conn_msg.kp_id1.push_back(outConn[i].kp_id1);
                conn_msg.kp_id2.push_back(outConn[i].kp_id2);
            }
            bag_out.write("connections", ros::Time::now(), conn_msg);
        }
        if(!ros::ok()){
            break;
        }
    }
    
    return 0;
};
