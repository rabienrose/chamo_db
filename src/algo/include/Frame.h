#ifndef FRAME_H
#define FRAME_H
#include<vector>
#include "MapPoint.h"
#include "ORBextractor.h"
#include <opencv2/opencv.hpp>

namespace CHAMO_DB{
    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64
    class MapPoint;
    class Frame{
    public:
        Frame();
        Frame(const Frame &frame);
        Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, cv::Mat &K, cv::Mat &distCoef);
        void ExtractORB(const cv::Mat &im);
        void SetPose(cv::Mat Tcw);
        void UpdatePoseMatrices();
        inline cv::Mat GetCameraCenter(){
            return mOw.clone();
        }
        inline cv::Mat GetRotationInverse(){
            return mRwc.clone();
        }
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
        std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;
    public:
        ORBextractor* mpORBextractorLeft;
        double mTimeStamp;
        cv::Mat mK;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;
        int N;
        std::vector<cv::KeyPoint> mvKeys;
        std::vector<cv::KeyPoint> mvKeysUn;
        cv::Mat mDescriptors;
        std::vector<MapPoint*> mvpMapPoints;
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
        cv::Mat mTcw;
        static long unsigned int nNextId;
        long unsigned int mnId;
        std::string bag_name;
        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        std::vector<float> mvScaleFactors;
        std::vector<float> mvInvScaleFactors;
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;
        static bool mbInitialComputations;
        int id_in_bag;
    private:
        void UndistortKeyPoints();
        void ComputeImageBounds(const cv::Mat &imLeft);
        void AssignFeaturesToGrid();
        cv::Mat mRcw;
        cv::Mat mtcw;
        cv::Mat mRwc;
        cv::Mat mOw; //==mtwc
    };
}

#endif // FRAME_H
