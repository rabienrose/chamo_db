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
#include <Initializer.h>
#include <Frame.h>
#include <Initializer.h>

class Connection{
public:
    int frame_id1;
    int kp_id1;
    int frame_id2;
    int kp_id2;
};


class SystemFront{
public:
    bool init(std::string bag_name_, std::string config_file){
        bag_name=bag_name_;
        loadCamConif(config_file);
        std::cout <<"fScaleFactor: "<<fScaleFactor<<std::endl;
        mpExtractor = new CHAMO_DB::ORBextractor(nFeatures,fScaleFactor,level,fast_thres, min_fast_thres);
    }
    bool processData(cv::Mat img, double timeStamp, std::vector<Connection>& outConn){
        cv::Ptr<cv::ORB> extractor=cv::ORB::create();
        std::vector<cv::KeyPoint> kps;
        cv::Mat mask;
        cv::Mat descs;
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::shared_ptr<CHAMO_DB::Frame> frame= std::make_shared<CHAMO_DB::Frame>(gray, timeStamp, mpExtractor, kMat, distMat);
        frame->bag_name = bag_name;
        cv::Mat debug_img= img.clone();
        for (int i=0; i<(int)frame->mvKeysUn.size();i++){
            cv::circle(debug_img, frame->mvKeysUn[i].pt, 1, cv::Scalar(255,0,0,255), 2);
        }
        cv::imshow("chamo", debug_img);
        cv::waitKey(-1);
        if(!frame_match){
            frame_match =  new CHAMO_DB::Initializer(*frame,1.0,200);
        }
        
        std::list<std::shared_ptr<CHAMO_DB::Frame>>::iterator iterator;
        for (iterator = frame_list.begin(); iterator != frame_list.end(); ++iterator) {
            cv::BFMatcher matcher;
            std::vector<cv::DMatch> matches;
            matcher.match(frame->mDescriptors, (*iterator)->mDescriptors, matches);
            std::vector<int> vMatches12;
            vMatches12.resize(frame->mDescriptors.rows);
            for( int j = 0; j < frame->mDescriptors.rows; j++ ){
                vMatches12.push_back(-1);
            }
            for( int j = 0; j < matches.size(); j++ ){
                if(matches[j].queryIdx >=vMatches12.size()){
                    std::cout<<"matches[j].matches overflow!!"<<std::endl;
                }
                vMatches12[matches[j].queryIdx]=matches[j].trainIdx;
            }
            
            cv::Mat R21;
            cv::Mat t21;
            std::vector<cv::Point3f> vP3D;
            std::vector<bool> vbTriangulated;
            frame_match->Initialize(**iterator, vMatches12, R21, t21, vP3D, vbTriangulated);
            if(vP3D.size()>0){
                std::cout<<vP3D[0]<<std::endl;
            }else{
                std::cout<<"no 3d points"<<std::endl;
            }
        }
        frame_list.push_back(frame);
    }
private:
    void loadCamConif(std::string config_file){
        cv::FileStorage fSettings(config_file, cv::FileStorage::READ);
        if (fSettings.isOpened())
        {
            kMat=cv::Mat::eye(3, 3, CV_32FC1);
            kMat.at<float>(0,0) = fSettings["Camera.fx"];
            kMat.at<float>(1,1) = fSettings["Camera.fy"];
            kMat.at<float>(0,2) = fSettings["Camera.cx"];
            kMat.at<float>(1,2) = fSettings["Camera.cy"];
            distMat=cv::Mat(1, 5, CV_32FC1);
            distMat.at<float>(0,0) = fSettings["Camera.k1"];
            distMat.at<float>(0,1) = fSettings["Camera.k2"];
            distMat.at<float>(0,4) = fSettings["Camera.k3"];

            distMat.at<float>(0,2) = fSettings["Camera.p1"];
            distMat.at<float>(0,3) = fSettings["Camera.p2"];
            nFeatures =fSettings["ORBextractor.nFeatures"];
            fScaleFactor =fSettings["ORBextractor.scaleFactor"];
            level =fSettings["ORBextractor.nLevels"];
            fast_thres =fSettings["ORBextractor.iniThFAST"];
            min_fast_thres=fSettings["ORBextractor.minThFAST"];
        }
        else
        {
            std::cout << "Failed to load setting file "<<std::endl;
        }
    }
    cv::Mat kMat;
    cv::Mat distMat;
    std::string bag_name;
    std::list<std::shared_ptr<CHAMO_DB::Frame>> frame_list;
    int nFeatures;
    float fScaleFactor;
    int level;
    int fast_thres;
    int min_fast_thres;
    CHAMO_DB::ORBextractor *mpExtractor;
    CHAMO_DB::Initializer* frame_match=NULL;
};



int main(int argc, char **argv){
    //float fx,fy,cx, cy, k1, k2, k3, p1, p2;
    ros::init(argc, argv, "cloud_expand");
    ros::NodeHandle nn;
    std::string bag_addr="/media/psf/Home/Documents/data/bag4chamodb";
    std::string bag_name="2018-03-20-23-43-14";
    std::string config_name="/media/psf/Home/Documents/code/chamo_db/src/cloud_expand/config/camera.yaml";
    rosbag::Bag bag;
    bag.open(bag_addr+"/"+bag_name+".bag",rosbag::bagmode::Read);
    rosbag::Bag bag_out;
    bag_out.open(bag_addr+"/chamo_re.bag", rosbag::bagmode::Write);
    SystemFront mySys;
    mySys.init(bag_name, config_name);
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
            mySys.processData(temp_img, simg->absTimestamp ,outConn);
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
