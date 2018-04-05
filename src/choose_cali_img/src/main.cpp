#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>
#include <stdio.h>
#include <cstdio>
#include <iomanip>

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

int main(int argc, char **argv){
    std::string blur_file_addr=argv[1];
    std::string group_file_addr=argv[2];
    std::ifstream blur_file(blur_file_addr);
    std::ifstream group_file(group_file_addr);
    std::map<std::string, float> blur_est_list;
    while(true){
        std::string str;
        std::getline(blur_file, str);
        if(str==""){
            break;
        }else{
            std::vector<std::string> split_re = split(str, ',');
            float est_value= (float)atof(split_re[1].c_str());
            std::pair<std::string, float> temp_pair;
            temp_pair.first=split_re[0];
            temp_pair.second=est_value;
            blur_est_list.insert(temp_pair);
        }
    }
    int current_gourp_id=0;
    std::vector<std::string> imgs_in_group;
    while(true){
        std::string str;
        std::getline(group_file, str);
        
        if(str==""){
            break;
        }else{
            std::vector<std::string> split_re = split(str, ',');
            int temp_gourp_it=atoi(split_re[0].c_str());
            //std::cout<<current_gourp_id<<" : "<<temp_gourp_it<<std::endl;
            imgs_in_group.push_back(split_re[1]);
            if (current_gourp_id!=temp_gourp_it){
                float min_blur=9999;
                std::string min_blur_img;
                for (int i=0; i<imgs_in_group.size();i++){
                    float min_temp=blur_est_list[imgs_in_group[i]];
                    if (min_temp<min_blur){
                        min_blur=min_temp;
                        min_blur_img=imgs_in_group[i];
                    }
                }
                std::cout<<min_blur_img<<std::endl;
                current_gourp_id=temp_gourp_it;
                imgs_in_group.clear();
            }
        }
    }
    return 0;
};
