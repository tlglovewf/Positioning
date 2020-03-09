#include "P_MapPoint.h"

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
        mMinDistance = 0;
        mMaxDistance = 0;
        mIndex = index;
    }
    PMapPoint::PMapPoint(const cv::Point3f &pt, PMap *pMap,u64 index):mpMap(pMap)
    {
        mPose = (Mat_<MATTYPE>(3,1) << pt.x,pt.y,pt.z);
        mMinDistance = 0;
        mMaxDistance = 0;
        mIndex = index;
    }
}