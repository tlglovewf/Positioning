#include "P_Map.h"

namespace Position
{
      //sort
    void IMap::SortFrames(KeyFrameVector &frames)
    {
        std::sort(frames.begin(), frames.end(),[](const IKeyFrame *pl, const IKeyFrame *pr)->bool
        {
            return pl->index() < pr->index();
        });
    }

    IKeyFrame* IMap::CreateKeyFrame(const std::shared_ptr<IMap> &pmap, const FrameData &data, const Mat &pose)
    {
        Position::IFrame *frame = new Position::PFrame(data,pmap->frameCount());
        frame->setPose(pose);
        return pmap->createKeyFrame(frame);
    }

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
        mpCurrent  = NULL;
    }
}