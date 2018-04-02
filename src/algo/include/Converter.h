#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

namespace CHAMO_DB
{

class Converter
{
public:
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);
};

}

#endif // CONVERTER_H
