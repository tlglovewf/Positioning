#include "P_ORBMap.h"

#include<mutex>

namespace Position
{
    ORBMap::ORBMap():mnMaxKFid(0),mpCurrent(NULL)
    {
    }

    void ORBMap::addKeyFrame(IKeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        mpCurrent = pKF;
        if(pKF->index() > mnMaxKFid)
            mnMaxKFid=pKF->index();
    }

    void ORBMap::addMapPoint(IMapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void ORBMap::rmMapPoint(IMapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the ORBMapPoint
    }

    void ORBMap::rmKeyFrame(IKeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // TODO: This only erase the pointer.
        // Delete the ORBMapPoint
    }

    void ORBMap::setReferenceMapPoints(const MapPtVector &vpMPs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    KeyFrameVector ORBMap::getAllFrames()
    {
        unique_lock<mutex> lock(mMutexMap);
        return KeyFrameVector(mspKeyFrames.begin(),mspKeyFrames.end());
    }

    MapPtVector ORBMap::getAllMapPts()
    {
        unique_lock<mutex> lock(mMutexMap);
        return MapPtVector(mspMapPoints.begin(),mspMapPoints.end());
    }

    IKeyFrame* ORBMap::currentKeyFrame()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpCurrent;
    }

    u64 ORBMap::mapptCount()
    {
        assert(NULL);
        return 0;
    }

    u64 ORBMap::frameCount()
    {
       assert(NULL);
       return 0;
    }

    //获取点、帧数量
    u64 ORBMap::mapPointsInMap() 
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }
    u64 ORBMap::keyFrameInMap() 
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    MapPtVector ORBMap::getReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    u64 ORBMap::getMaxKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void ORBMap::clear()
    {
        for(MapPtSet::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
            delete *sit;

        for(KeyFmSet::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mpCurrent = NULL;
        // mvpKeyFrameOrigins.clear();
    }
}
