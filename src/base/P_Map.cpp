#include "P_Map.h"

namespace Position
{
    void PMap::clear()
    {
        for(KeyFmSetIter it = mMapFms.begin(); it != mMapFms.end(); ++it)
        {
            delete *it;
        }
        mMapFms.clear();

        for(MapPtSetIter it = mMapPts.begin(); it != mMapPts.end(); ++it)
        {
            delete *it;
        }
        mMapPts.clear();
        mMaxFmId = 0;
        mnFrameCnt = 0;
        mnMapPtCnt = 0;
        mMaxFmId   = 0;
    }
}