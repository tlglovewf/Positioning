

#ifndef _ORBMAP_H_
#define _ORBMAP_H_

#include "P_ORBMapPoint.h"
#include "P_ORBKeyFrame.h"
#include <set>

#include <mutex>



namespace Position
{

class ORBMapPoint;
class ORBKeyFrame;

class Map
{
public:
    Map();

    void AddKeyFrame(ORBKeyFrame* pKF);
    void AddMapPoint(ORBMapPoint* pMP);
    void EraseMapPoint(ORBMapPoint* pMP);
    void EraseKeyFrame(ORBKeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<ORBMapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<ORBKeyFrame*> GetAllKeyFrames();
    std::vector<ORBMapPoint*> GetAllMapPoints();
    std::vector<ORBMapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<ORBKeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<ORBMapPoint*> mspMapPoints;
    std::set<ORBKeyFrame*> mspKeyFrames;

    std::vector<ORBMapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

}

#endif // _ORBMAP_H_
