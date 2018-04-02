#include "MapPoint.h"

namespace CHAMO_DB{
    long unsigned int MapPoint::nNextId=0;

    MapPoint::MapPoint(const cv::Mat &Pos, Frame* pFrame, const int &idxF): nObs(0){
        Pos.copyTo(mWorldPos);
        mnId=nNextId++;
    }

    void MapPoint::SetWorldPos(const cv::Mat &Pos){
        Pos.copyTo(mWorldPos);
    }

    cv::Mat MapPoint::GetWorldPos(){
        return mWorldPos.clone();
    }

    void MapPoint::AddObservation(Frame* pKF, size_t idx){
        if(mObservations.count(pKF))
            return;
        mObservations[pKF]=idx;
        nObs++;
    }

    void MapPoint::EraseObservation(Frame* pKF){
        if(mObservations.count(pKF)){
            nObs--;
            mObservations.erase(pKF);
        }
    }

    std::map<Frame*, size_t> MapPoint::GetObservations(){
        return mObservations;
    }

    int MapPoint::Observations(){
        return nObs;
    }

    int MapPoint::GetIndexInKeyFrame(Frame *pKF){
        if(mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    bool MapPoint::IsInKeyFrame(Frame *pKF){
        return (mObservations.count(pKF));
    }
} 
