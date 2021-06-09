#include "P_MapPoint.h"
#include "P_Map.h"

namespace Position
{
    //构造
    PMapPoint::PMapPoint(const cv::Mat &pose, PMap *pMap,u64 index):mpMap(pMap),mbBad(false)
    {
        if(pose.type() != MATCVTYPE)
        {
            pose.convertTo(mPose,MATCVTYPE);
        }
        mPose = pose.rowRange(0,3);
        mIndex = index;
    }
    PMapPoint::PMapPoint(const cv::Point3f &pt, PMap *pMap,u64 index):mpMap(pMap)
    {
        mPose = (Mat_<MATTYPE>(3,1) << pt.x,pt.y,pt.z);
        mIndex = index;
    }
    //设置坏点
    void PMapPoint::setBadFlag() 
    {
        for(KeyFrameMap::iterator mit=mObsers.begin(), mend=mObsers.end(); mit!=mend; mit++)
        {
            IKeyFrame* pKF = mit->first;
            pKF->rmMapPoint(mit->second);
        }
        mObsers.clear();
        mpMap->rmMapPoint(this);
        mbBad = true;
    }
}