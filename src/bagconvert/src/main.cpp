#include <string>
#include <iostream>
#include <boost/graph/graph_concepts.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <boost/filesystem.hpp>

using namespace std;

struct imu_data{
    double time;
    double ax,ay,az,gx,gy,gz;
};

void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

int main(int argc, char **argv) {

    // Debug message
    ROS_INFO("Starting up");

    // Check if there is a path to a dataset
    if(argc < 2) {
        ROS_ERROR("Error please specify a rosbag file");
        ROS_ERROR("Command Example: rosrun bagconvert bagconvert <rosbag> <topic>");
        return EXIT_FAILURE;
    }

    // Startup this node
    ros::init(argc, argv, "bagconvert");

    // Parse the input
    string pathBag = argv[1];
    string imuTopic = "/imu0";

    // Get path
    boost::filesystem::path p(pathBag);
    string pathParent = p.parent_path().string();
    string pathMat;
    if(!pathParent.empty()) {
        pathMat = pathParent+"/"+p.stem().string()+".mat";
    } else {
        pathMat = p.stem().string()+".mat";
    }


    // Load rosbag here, and find messages we can play
    rosbag::Bag bag;
    bag.open(pathBag, rosbag::bagmode::Read);
    bool save_direct=true;

    // We should load the bag as a view
    // Here we go from beginning of the bag to the end of the bag
    rosbag::View view(bag);

    // Debug
    ROS_INFO("BAG Path is: %s", pathBag.c_str());
    ROS_INFO("MAT Path is: %s", pathMat.c_str());
    ROS_INFO("Reading in rosbag file...");

    // Our data vector
    vector<double> dataIMU = vector<double>();
    std::ofstream outFile;
    outFile.open("imu.txt");
    std::ofstream outFile_img;
    outFile_img.open("img.txt");
    // Step through the rosbag and send to algo methods
    unsigned int msg_count=0;
    std::vector<imu_data> imu_data_list;
    for (const rosbag::MessageInstance& m : view) {
        sensor_msgs::ImagePtr simg = m.instantiate<sensor_msgs::Image>();
        if(simg!=NULL){
            std::stringstream ss;
            ss<<std::setprecision(20)<<simg->header.stamp.toSec()<<std::endl;
            outFile_img<<ss.str();
        }
        sensor_msgs::Imu::ConstPtr s1 = m.instantiate<sensor_msgs::Imu>();
        if (s1 != NULL && m.getTopic() == imuTopic) {
            if(msg_count==0){
                msg_count=s1->header.seq;
            }
            if(msg_count!=s1->header.seq){
                std::cout<<"lose one imu data"<<std::endl;
            }
            
            imu_data item;
            item.time=s1->header.stamp.toSec();
            std::cout<<std::setprecision(20)<<item.time<<std::endl;
            item.ax=s1->linear_acceleration.x;
            item.ay=s1->linear_acceleration.y;
            item.az=s1->linear_acceleration.z;
            item.gx=s1->angular_velocity.x;
            item.gy=s1->angular_velocity.y;
            item.gz=s1->angular_velocity.z;
            static double last_time=-1;
            static unsigned int last_id=0;
            if(last_time==-1){
                last_time=item.time;
                last_id=s1->header.seq;
            }else{
                if(item.time<last_time){
                    std::cout<<"time order issue!!"<<std::endl;
                    continue;
                }
                if(s1->header.seq<=last_id){
                    //std::cout<<"id order issue!!"<<std::endl;
                    //continue;
                }
                last_time=item.time;
                last_id=s1->header.seq;
            }
            if(save_direct){
                std::stringstream ss;
                ss<<std::setprecision(20)<<item.time<<","<<item.ax<<","<<item.ay<<","<<item.az<<","<<item.gx<<","<<item.gy<<","<<item.gz<<std::endl;
                outFile<<ss.str();
            }else{
                imu_data_list.push_back(item);
            }
            msg_count++;
        }
    }
    std::cout<<"get "<<msg_count<<" imu."<<std::endl;
    if(!save_direct){
        double process_time=imu_data_list[0].time;
        double delta_time=0.005;
        unsigned int cur_index=0;
        while(true){
            if(cur_index>imu_data_list.size()/4){
                break;
            }
            for(unsigned int i=cur_index;i<imu_data_list.size()-1;i++){
                if(imu_data_list[i].time<=process_time && process_time<imu_data_list[i+1].time){
                    double inter_ax, inter_ay,inter_az,inter_gx,inter_gy,inter_gz;
                    interDouble(imu_data_list[i].ax, imu_data_list[i+1].ax, imu_data_list[i].time, imu_data_list[i+1].time, inter_ax, process_time);
                    interDouble(imu_data_list[i].ay, imu_data_list[i+1].ay, imu_data_list[i].time, imu_data_list[i+1].time, inter_ay, process_time);
                    interDouble(imu_data_list[i].az, imu_data_list[i+1].az, imu_data_list[i].time, imu_data_list[i+1].time, inter_az, process_time);
                    interDouble(imu_data_list[i].gx, imu_data_list[i+1].gx, imu_data_list[i].time, imu_data_list[i+1].time, inter_gx, process_time);
                    interDouble(imu_data_list[i].gy, imu_data_list[i+1].gy, imu_data_list[i].time, imu_data_list[i+1].time, inter_gy, process_time);
                    interDouble(imu_data_list[i].gz, imu_data_list[i+1].gz, imu_data_list[i].time, imu_data_list[i+1].time, inter_gz, process_time);
                    cur_index=i-1;
                    std::stringstream ss;
                    ss<<std::setprecision(20)<<process_time<<","<<inter_ax<<","<<inter_ay<<","<<inter_az<<","<<inter_gx<<","<<inter_gy<<","<<inter_gz<<std::endl;
                    //std::cout<<std::setprecision(20)<<m.getTime().toSec()<<","<<s1->linear_acceleration.x<<","<<s1->linear_acceleration.y<<","<<s1->linear_acceleration.z<<","<<s1->angular_velocity.x<<","<<s1->angular_velocity.y<<","<<s1->angular_velocity.z<<std::endl;
                    outFile<<ss.str();
                    break;
                }
            }
            process_time=process_time+delta_time;
        }
    }
    
    return EXIT_SUCCESS;
}


