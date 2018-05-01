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

cv::Mat getImgFromBag(int frameID, std::string bag_name){
    rosbag::Bag bag;
    bag.open(bag_name,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("img_chamo");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        save_bag::img_chamoConstPtr simg = m.instantiate<save_bag::img_chamo>();
        if(simg!=NULL){
            if(img_count==frameID){
                cv::Mat temp_img;
                temp_img = cv::imdecode(simg->jpg, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);
                cv::Mat gray;
                cv::cvtColor(temp_img, gray, cv::COLOR_BGR2GRAY);
                return gray;
            }
            img_count++;
        }
    }
    bag.close();
}

void showMatches(cv::Mat img1, cv::Mat img2, std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2, std::vector<cv::DMatch>& good_matches){
    cv::Mat track_canvas;
    img2.copyTo(track_canvas);
    cv::cvtColor(track_canvas,track_canvas,CV_GRAY2RGB);
    for(int i=0; i< keypoints2.size();i++){
        cv::circle(track_canvas, keypoints2[i].pt, 1, cv::Scalar(0,0,0),2);
    }
    for(int i=0; i< good_matches.size();i++){
        int kpInd1=good_matches[i].queryIdx;
        int kpInd2=good_matches[i].trainIdx;
        cv::line(track_canvas, keypoints1[kpInd1].pt, keypoints2[kpInd2].pt, CV_RGB(255,0,0));
        cv::circle(track_canvas, keypoints2[kpInd2].pt, 1, CV_RGB(255,0,0),2);
    }
    cv::imshow("track_canvas", track_canvas);
    
    cv::Size size;
    size.height =img1.rows;
    size.width =img1.cols;
    cv::Mat canvas(size.height*2, size.width, img1.type());
    img1.copyTo(canvas.rowRange(0, size.height));
    img2.copyTo(canvas.rowRange(size.height, size.height*2));
    //cv::cvtColor(canvas,canvas,CV_RGB2GRAY);
    cv::cvtColor(canvas,canvas,CV_GRAY2RGB);
    for(int i=0; i< good_matches.size();i++){
        int kpInd1=good_matches[i].queryIdx;
        int kpInd2=good_matches[i].trainIdx;
        cv::Point2f siftpt = keypoints2[kpInd2].pt;
        siftpt.y=siftpt.y+size.height;
        cv::line(canvas, keypoints1[kpInd1].pt, siftpt, CV_RGB(255,255,255));
        
        cv::Scalar clr(0,rand() % 255,rand() % 255);
        cv::circle(canvas, keypoints1[kpInd1].pt, 1, clr,2);
        cv::circle(canvas, siftpt, 1, clr,2);
    }
    cv::Size sizeCan;
    sizeCan.height =canvas.rows*1;
    sizeCan.width =canvas.cols*1;
    cv::resize(canvas, canvas, sizeCan);
    cv::imshow("img1", canvas);
    
    cv::waitKey(-1);
}

class Connection{
public:
    int frame_id1;
    int kp_id1;
    int frame_id2;
    int kp_id2;
};


class SystemFront{
public:
    bool init(std::string bag_name_, std::string config_file, std::string bag_addr_){
        bag_name=bag_name_;
        bag_addr=bag_addr_;
        loadCamConif(config_file);
        std::cout <<"fScaleFactor: "<<fScaleFactor<<std::endl;
        mpExtractor = new CHAMO_DB::ORBextractor(nFeatures,fScaleFactor,level,fast_thres, min_fast_thres);
    }
    bool processData(int id_in_bag, cv::Mat img, double timeStamp, std::vector<Connection>& outConn){
        cv::Ptr<cv::ORB> extractor=cv::ORB::create();
        std::vector<cv::KeyPoint> kps;
        cv::Mat mask;
        cv::Mat descs;
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::shared_ptr<CHAMO_DB::Frame> frame= std::make_shared<CHAMO_DB::Frame>(gray, timeStamp, mpExtractor, kMat, distMat);
        frame->bag_name = bag_name;
        frame->id_in_bag=id_in_bag;
        cv::Mat debug_img= img.clone();
        for (int i=0; i<(int)frame->mvKeys.size();i++){
            cv::circle(debug_img, frame->mvKeys[i].pt, 1, cv::Scalar(255,0,0,255), 2);
        }
        cv::imshow("chamo", debug_img);
        //cv::waitKey(-1);
        if(frame_match!=NULL){
            delete frame_match;
            frame_match = NULL;
        }
        frame_match = new CHAMO_DB::Initializer(*frame,1.0,200);
        std::list<std::shared_ptr<CHAMO_DB::Frame>>::iterator iterator;
        for (iterator = frame_list.begin(); iterator != frame_list.end(); ++iterator) {
            if (frame_list.size()<10){
                continue;
            }
            cv::BFMatcher matcher;
            std::vector<cv::DMatch> matches;
            matcher.match(frame->mDescriptors, (*iterator)->mDescriptors, matches);
            std::sort(matches.begin(), matches.end());
            const int ptsPairs = std::min(1000, (int)(matches.size() * 0.4f));
            std::vector<int> vMatches12;
            for( int j = 0; j < frame->mDescriptors.rows; j++ ){
                vMatches12.push_back(-1);
            }
            std::vector<cv::DMatch> good_matches;
            for( int j = 0; j < ptsPairs; j++ )
            {
                if(matches[j].queryIdx >=vMatches12.size()){
                    std::cout<<"matches[j].matches overflow!!"<<std::endl;
                }
                good_matches.push_back(matches[j]);
                vMatches12[matches[j].queryIdx]=matches[j].trainIdx;
            }

            cv::Mat R21;
            cv::Mat t21;
            std::vector<cv::Point3f> vP3D;
            std::vector<bool> vbTriangulated;
            
            frame_match->Initialize(**iterator, vMatches12, R21, t21, vP3D, vbTriangulated);
            if(vP3D.size()>0){
                std::cout<<"3d points count: "<<vP3D.size()<<std::endl;
            }else{
                std::cout<<"no 3d points"<<std::endl;
            }
            std::string bag_full_addr=bag_addr+"/"+bag_name+".bag";
            cv::Mat img2=getImgFromBag((*iterator)->id_in_bag, bag_full_addr);
            showMatches(gray, img2, frame->mvKeys, (*iterator)->mvKeys, good_matches);
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
    std::string bag_addr;
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
    mySys.init(bag_name, config_name, bag_addr);
    std::vector<std::string> topics;
    topics.push_back("img_chamo");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        save_bag::img_chamoConstPtr simg = m.instantiate<save_bag::img_chamo>();
        if(simg!=NULL){
            cv::Mat temp_img;
            temp_img = cv::imdecode(simg->jpg, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);
            std::vector<Connection> outConn;
            mySys.processData(img_count, temp_img, simg->absTimestamp ,outConn);
            cloud_expand::connection conn_msg;
            for (int i=0;i<(int)outConn.size();i++){
                conn_msg.frame_id1.push_back(outConn[i].frame_id1);
                conn_msg.frame_id2.push_back(outConn[i].frame_id2);
                conn_msg.kp_id1.push_back(outConn[i].kp_id1);
                conn_msg.kp_id2.push_back(outConn[i].kp_id2);
            }
            bag_out.write("connections", ros::Time::now(), conn_msg);
            img_count++;
        }
        if(!ros::ok()){
            break;
        }
    }
    
    return 0;
};
