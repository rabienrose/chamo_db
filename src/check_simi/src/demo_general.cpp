#include <iostream>
#include <vector>
#include "DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "DescManip.h"

std::vector<cv::Mat> loadFeatures( std::vector<std::string>& path_to_images,std::string descriptor="") throw (std::exception){
    //select detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptor=="orb")        fdetector=cv::ORB::create();
    else if (descriptor=="brisk") fdetector=cv::BRISK::create();
    else if (descriptor=="akaze") fdetector=cv::AKAZE::create();
    else throw std::runtime_error("Invalid descriptor");
    assert(!descriptor.empty());
    std::vector<cv::Mat>    features;
    //std::cout << "Extracting   features..." << std::endl;
    for(size_t i = 0; i < path_to_images.size(); ++i)
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        //std::cout<<"reading image: "<<path_to_images[i]<<std::endl;
        cv::Mat image = cv::imread(path_to_images[i], 0);
        if(image.empty())throw std::runtime_error("Could not open image"+path_to_images[i]);
        fdetector->detectAndCompute(image, cv::Mat(), keypoints, descriptors);
        features.push_back(descriptors);
    }
    return features;
}

void testVocCreation(const std::vector<cv::Mat> &features)
{
    const int k = 9;
    const int L = 3;
    const DBoW3::WeightingType weight =  DBoW3::TF_IDF;
    const DBoW3::ScoringType score =  DBoW3::L1_NORM;
    DBoW3::Vocabulary voc(k, L, weight, score);
    voc.create(features);
    std::cout << "Vocabulary information: " << std::endl<< voc << std::endl << std::endl;
    voc.save("small_voc.bin");
}

int main(int argc,char **argv)
{
    std::string cmd_flag=argv[1];
    std::string img_root = argv[2];
    std::string file_list_filename = argv[3];
    std::ifstream file_file_list_file(file_list_filename);
    std::vector<std::string> file_list_vec;
    std::vector<std::string> file_name_vec;
    std::vector<bool> img_class_status;
    while(true){
        std::string str;
        std::getline(file_file_list_file, str);
        if(str==""){
            break;
        }else{
            file_list_vec.push_back(img_root+"/"+str);
            file_name_vec.push_back(str);
            img_class_status.push_back(false);
        }
    }
    try{
        if(cmd_flag=="gen"){
            std::vector<cv::Mat> features = loadFeatures(file_list_vec, "orb");  
            testVocCreation(features);
        }else if(cmd_flag=="test"){
            std::vector<cv::Mat> features = loadFeatures(file_list_vec, "orb");  
            DBoW3::Vocabulary voc("small_voc.bin");
            DBoW3::Database db(voc, false, 0);
            for(size_t i = 0; i < features.size(); i++){
                db.add(features[i]);
            }
            std::string test_img_addr = argv[4];
            std::vector<std::string> file_temp;
            file_temp.push_back(test_img_addr);
            std::vector<cv::Mat> decs = loadFeatures(file_temp, "orb");
            DBoW3::QueryResults ret;
            for(size_t i = 0; i < decs.size(); i++){
                db.query(decs[0], ret, 20);
                for (size_t re_i=0; re_i<ret.size();re_i++){
                    std::string match_img_name = file_list_vec[ret[re_i].Id];
                    cv::Mat re_img = cv::imread(match_img_name);
                    std::cout << "img: " << match_img_name << ". " << ret[re_i].Score << std::endl;
                    cv::imshow("chamo", re_img);
                    cv::waitKey(-1);
                }
            }
        }else if(cmd_flag=="blur"){
            //std::cout<<"blur compare, img: "<<file_list_vec.size()<<std::endl;
            std::vector<cv::Mat> blur_imgs;
            std::vector<cv::Mat> blur_imgs_f;
            for(size_t i=0; i<file_list_vec.size(); i++){
                std::string img_addr= file_list_vec[i];
                cv::Mat raw_img = cv::imread(img_addr);
                cv::Mat gray_img = cv::imread(img_addr);
                cv::cvtColor(raw_img, gray_img, CV_BGR2GRAY);
                cv::Mat blur_img;
                double sigma=10;
                GaussianBlur(gray_img, blur_img, cv::Size(99, 99), sigma, sigma);
                blur_imgs.push_back(blur_img);
                cv::Mat test_img_f;
                blur_img.convertTo(test_img_f, CV_32FC1);
                blur_imgs_f.push_back(test_img_f);
                
            }
            int group_id=0;
            for(size_t test_img_id=0;test_img_id<blur_imgs.size();test_img_id++){
                if(img_class_status[test_img_id]==true){
                    continue;
                }
                for(size_t i=0; i<blur_imgs.size();i++){
                    if(img_class_status[i]==true){
                        continue;
                    }
                    cv::Mat diff_img;
                    cv::absdiff(blur_imgs_f[i], blur_imgs_f[test_img_id], diff_img);
                    cv::Mat mean;
                    cv::Mat stddev;
                    cv::meanStdDev(diff_img, mean, stddev);
                    if (mean.at<double>(0,0)<10){
                        img_class_status[i]= true;
                        std::cout<<group_id<<","<<file_name_vec[i]<<","<<mean.at<double>(0,0)<<std::endl;
                    }
                    //cv::imshow("chamo", blur_imgs[i]);
                    //cv::imshow("chamo_blur", blur_img);
                    //cv::waitKey(-1);
                }
                group_id++;
            }
        }
    }catch(std::exception &ex){
        std::cerr<<ex.what()<<std::endl;
    }

    return 0;
}
