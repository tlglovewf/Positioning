

#ifndef _ORBMAP_H_
#define _ORBMAP_H_

#include "P_Interface.h"

namespace Position
{
    class ORBMap : public IMap
    {
    public:
        ORBMap();

        // //创建关键帧
        virtual IKeyFrame* createKeyFrame(IFrame *frame) 
        {
            assert(NULL);
            return NULL;
        }
        //创建地图点
        virtual IMapPoint* createMapPoint(const cv::Mat &pose) 
        {
            assert(NULL);
            return NULL;
        }
        virtual IMapPoint* createMapPoint(const cv::Point3f &pose)
        {
            assert(NULL);
            return NULL;
        }

        //加入/移除地图点
        virtual void addMapPoint(IMapPoint *pMp);
        virtual void rmMapPoint(IMapPoint *pMp) ;

        //加入/移除关键帧
        virtual void addKeyFrame(IKeyFrame *pKF);
        virtual void rmKeyFrame(IKeyFrame *pKF) ;

        //最大帧号
        virtual u64 getMaxKFid() ;

        //获取所有地图点
        virtual MapPtVector getAllMapPts();

        //获取所有帧
        virtual KeyFrameVector getAllFrames();

        //清空
        virtual void clear() ;

        //设置最近关联点
        virtual void setReferenceMapPoints(const MapPtVector &vpMPs);

        //获取最近关联点
        virtual MapPtVector getReferenceMapPoints();

        //获取帧，点计数
        virtual u64 frameCount() ;
        virtual u64 mapptCount() ;

        //获取点、帧数量
        virtual u64 mapPointsInMap();
        virtual u64 keyFrameInMap();

         //用于多线程 地图更新锁
        virtual std::mutex& mapUpdateMutex()
        {
            return mMutexMapUpdate;
        }

        // KeyFrameVector  mvpKeyFrameOrigins;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;

        //当前帧
        virtual IKeyFrame* currentKeyFrame();

    protected:
        MapPtSet               mspMapPoints;
        KeyFmSet               mspKeyFrames;

        MapPtVector            mvpReferenceMapPoints;

        IKeyFrame             *mpCurrent;

        u64 mnMaxKFid;

        std::mutex mMutexMap;
        std::mutex mMutexMapUpdate;
    };
#define ORBMAP(P) dynamic_cast<Position::ORBMap*>(P)
}

#endif // _ORBMAP_H_
