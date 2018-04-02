#ifndef MAPPOINT_H
#define MAPPOINT_H
#include"Frame.h"
#include<opencv2/core/core.hpp>
#include <map>
namespace CHAMO_DB{
    class Frame;
    class MapPoint{
    public:
        MapPoint(const cv::Mat &Pos, Frame* pFrame, const int &idxF);

        void SetWorldPos(const cv::Mat &Pos);
        cv::Mat GetWorldPos();

        std::map<Frame*,size_t> GetObservations();
        void AddObservation(Frame* pKF,size_t idx);
        void EraseObservation(Frame* pKF);
        int Observations();

        int GetIndexInKeyFrame(Frame* pKF);
        bool IsInKeyFrame(Frame* pKF);
    public:
        long unsigned int mnId;
        static long unsigned int nNextId;
    protected:    
        int nObs;
        cv::Mat mWorldPos;
        std::map<Frame*,size_t> mObservations;
    };
} 

#endif // MAPPOINT_H
