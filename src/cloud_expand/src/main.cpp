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
    std::string bag_name;
    cv::Mat descs;
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
    bool init(std::string bag_name_){
        bag_name=bag_name_;
    }
    bool processData(int frame_id, cv::Mat img, std::vector<Connection>& outConn){
        cv::Ptr<cv::ORB> extractor=cv::ORB::create();
        std::vector<cv::KeyPoint> kps;
        cv::Mat mask;
        cv::Mat descs;
        extractor->detectAndCompute(img, mask, kps, descs);
        Frame frame;
        frame.bag_name = bag_name;
        frame.frame_id = frame_id;
        cv::Mat debug_img= img.clone();
        frame.descs = descs;
        for (int i=0; i<(int)kps.size();i++){
            frame.kps.push_back(kps[i]);
            //cv::circle(debug_img, kps[i].pt, 1, cv::Scalar(255,0,0,255), 2);
        }
        
        matchTwoFrame();
        
        //cv::imshow("chamo", debug_img);
        //cv::waitKey(-1);
        
    }
private:
    void matchTwoFrame(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2, std::vector<cv::DMatch> out_matches){
        cv::BFMatcher matcher;
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);
        std::sort(matches.begin(), matches.end());
        std::vector<cv::DMatch> good_matches;
        const int ptsPairs = std::min(1000, (int)(matches.size() * 0.4f));
        for( int j = 0; j < ptsPairs; j++ )
        {
            good_matches.push_back(matches[j]);
        }
        
        std::vector<cv::Point2f> points1(good_matches.size());
        std::vector<cv::Point2f> points2(good_matches.size());
        std::vector<int> points1_ind(good_matches.size());
        std::vector<int> points2_ind(good_matches.size());
        for(int i=0; i< good_matches.size();i++){
            int kpInd1=good_matches[i].queryIdx;
            int kpInd2=good_matches[i].trainIdx;
            points1_ind[i]=kpInd1;
            points2_ind[i]=kpInd2;
            points1[i] =keypoints1[kpInd1].pt;
            points2[i] =keypoints2[kpInd2].pt;
        }
        cv::Mat inliers;
        cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 1., 0.99, inliers);
        for( int j = 0; j < inliers.rows; j++ )
        {
            if(inliers.at<unsigned char>(j)==1){
                out_matches.push_back(good_matches[j]);
            }
        }
    }
    
    std::string bag_name;
    std::list<Frame> frame_list;
};

int main(int argc, char **argv){
    //float fx,fy,cx, cy, k1, k2, k3, p1, p2;
    ros::init(argc, argv, "cloud_expand");
    ros::NodeHandle nn;
    std::string bag_addr="/media/psf/Home/Documents/data/bag4chamodb";
    std::string bag_name="2018-03-20-23-43-14"
    rosbag::Bag bag;
    bag.open(bag_addr,rosbag::bagmode::Read);
    rosbag::Bag bag_out;
    bag_out.open(bag_addr+"/"+bag_name+".bag", rosbag::bagmode::Write);
    SystemFront mySys;
    mySys.init(bag_name);
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
            mySys.processData(img_count, temp_img, outConn);
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
