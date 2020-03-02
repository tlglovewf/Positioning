#include "P_MapPoint.h"

namespace Position
{
    u64  PMapPoint::s_nIndexCount = 0;

    //构造
    PMapPoint::PMapPoint(const cv::Mat &pose, PMap *pMap):mpMap(pMap),mbBad(false)
    {
        if(pose.type() != CV_64F)
        {
            pose.convertTo(mPose,CV_64F);
        }
        mPose = pose.rowRange(0,3);
        mIndex = s_nIndexCount++;
    }
    PMapPoint::PMapPoint(const cv::Point3f &pt, PMap *pMap):mpMap(pMap)
    {
        mPose = (Mat_<double>(3,1) << pt.x,pt.y,pt.z);
        mIndex = s_nIndexCount++;
    }
}