#include "P_MapSerializor.h"
#include "P_Factory.h"
#include "P_Map.h"
namespace Position
{
    /******************************************************/
    /****************** ISerialization ********************/
    /******************************************************/

    MapSerManager::MapSerManager() : mpTracSer(NULL), mpPtSer(NULL)
    {
        mpTracSer = new PMapTraceSer();
        mpPtSer = new PMapPointsSer();
    }
    //设置持久化类型
    void MapSerManager::SetSerType(eMapTraceSerType trtype, eMapPointSerType epttype)
    {
        if (eDefaultTraceSer != trtype)
        {
            // switch (trtype)
            // {
            //     //add more ...
            // }
        }

        if (eDefaultPtSer != epttype)
        {
            // switch (epttype)
            // {
            //     //add more ...
            // }
        }
    }

    //显示地图
    void MapSerManager::displayMap(const std::shared_ptr<IConfig> &pCfg, const std::string &trac, const std::string &mpts)
    {
    #ifdef USE_VIEW
        //可视化帧数据
        std::shared_ptr<Position::IViewer> pv(GETVIEWER());
        std::shared_ptr<Position::IMap> pmap(new Position::PMap());
        setMap(pmap);
        if(!trac.empty())
            mpTracSer->loadMap(trac);
        if(!mpts.empty())
            mpPtSer->loadMap(mpts);
        pv->setMap(pmap);
        pv->renderLoop();
    #endif
    }
    //融合地图  secMap -> baseMap
    void MapSerManager::combineMap(std::shared_ptr<IMap> &baseMap, const std::shared_ptr<IMap> &secMap)
    {
        KeyFrameVector fms = baseMap->getAllFrames();
        KeyFrameVector sfms = secMap->getAllFrames();

        //获取第一个地图最后一帧 当做第二个地图的起止位姿
        IKeyFrame *pLast = *fms.rbegin();

        for (size_t i = 0; i < sfms.size(); ++i)
        {
            IFrame *frame = new PFrame(sfms[i]->getData(), baseMap->frameCount());
            frame->setPose(sfms[i]->getPose() * pLast->getPose());
            baseMap->addKeyFrame(baseMap->createKeyFrame(frame));
        }
        //写地图点
        MapPtVector mapts = secMap->getAllMapPts();
        for (size_t i = 0; i < mapts.size(); ++i)
        {
            cv::Mat post = mapts[i]->getWorldPos();
            Mat pt = Mat(4, 1, MATCVTYPE);
            post.copyTo(pt.rowRange(0, 3));
            pt.at<double>(3) = 1.0;
            mapts[i]->setWorldPos(pt + pLast->getCameraCenter());
            baseMap->addMapPoint(mapts[i]);
        }
    }
} // namespace  Position
