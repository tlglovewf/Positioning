#include "P_Factory.h"

#include "P_BlockMatcher.h"
#include "P_ORBFeature.h"
#include "P_SiftFeature.h"
#include "P_UniformDistriFeature.h"

#include "P_FeatureMatcher.h"

#include "P_PoseSolver.h"
#include "P_Optimizer.h"
#include "P_Positioner.h"

#include "P_Checker.h"

#include "P_PangolinViewer.h"
#include "P_UniformVTrajProcesser.h"
#include "P_MultiVisionTrajProcesser.h"
#include "P_SfmVisonTrajProcesser.h"

#include "P_IOHelper.h"


namespace Position
{
    /******************************************************/
    /****************** ISerialization ********************/
    /******************************************************/

    MapSerManager::MapSerManager():mpTracSer(NULL),mpPtSer(NULL)
    {
        mpTracSer =  new PMapTraceSer(); 
        mpPtSer   =  new PMapPointsSer(); 
    }
    //设置持久化类型
    void MapSerManager::SetSerType(eMapTraceSerType trtype, eMapPointSerType epttype)
    {
        if(eDefaultTraceSer != trtype)
        {
            // switch (trtype)
            // {
            //     //add more ...
            // }
        }

        if( eDefaultPtSer != epttype)
        {
            // switch (epttype)
            // {
            //     //add more ...
            // }
        }
    }

    //显示地图
    void MapSerManager::displayMap(const std::shared_ptr<IConfig> &pCfg, const std::string &trac,const std::string &mpts)
    {
#ifdef USE_VIEW
        //可视化帧数据
        std::shared_ptr<Position::IViewer> pv(GETVIEWER());
        std::shared_ptr<Position::IMap> pmap(new Position::PMap());
        setMap(pmap);
        mpTracSer->loadMap(trac);
        mpPtSer->loadMap(mpts);
        pv->setMap(pmap);
        pv->renderLoop();
#endif
    }
    //融合地图  secMap -> baseMap
    void MapSerManager::combineMap(std::shared_ptr<IMap> &baseMap, const std::shared_ptr<IMap> &secMap)
    {
        KeyFrameVector fms  = baseMap->getAllFrames();
        KeyFrameVector sfms = secMap->getAllFrames();

        //获取第一个地图最后一帧 当做第二个地图的起止位姿
        IKeyFrame *pLast = *fms.rbegin(); 

        for(size_t i = 0;i < sfms.size(); ++i)
        {
            IFrame *frame = new PFrame(sfms[i]->getData(),baseMap->frameCount());
            frame->setPose(sfms[i]->getPose() * pLast->getPose());
            baseMap->addKeyFrame(baseMap->createKeyFrame(frame));
        }
        //写地图点
        MapPtVector mapts =  secMap->getAllMapPts();
        for(size_t i = 0; i < mapts.size(); ++i)
        {
            cv::Mat post = mapts[i]->getWorldPos();
            Mat pt = Mat(4,1,MATCVTYPE);
            post.copyTo(pt.rowRange(0,3));
            pt.at<double>(3) = 1.0;
            mapts[i]->setWorldPos( pt + pLast->getCameraCenter());
            baseMap->addMapPoint(mapts[i]);
        }
    }  


#define FACTORYINSTANCEDECLARE(F)  F##Factory* F##Factory::Instance(){\
        static F##Factory item;\
        return &item;}


    FACTORYINSTANCEDECLARE(IFeature)
    FACTORYINSTANCEDECLARE(IFeatureMatcher)
    FACTORYINSTANCEDECLARE(IPoseSolver)
    FACTORYINSTANCEDECLARE(ITrajProcesser)
    FACTORYINSTANCEDECLARE(IOptimizer)

#if USE_VIEW
    FACTORYINSTANCEDECLARE(IViewer)
#endif


#define INSERT_FACTORY_ITEM(F,T) { std::shared_ptr< Position::IBaseFactory<Position::T> > p(new Position::F##Factory);\
                                mItems.insert(make_pair(p->name(),p));}

    IFeatureFactory::IFeatureFactory()
    {
        INSERT_FACTORY_ITEM(ORBFeature,IFeature)
        INSERT_FACTORY_ITEM(SiftFeature,IFeature)
        INSERT_FACTORY_ITEM(UniformDistriFeature,IFeature)
    }

    IFeatureMatcherFactory::IFeatureMatcherFactory()
    {
        INSERT_FACTORY_ITEM(HanMingMatcher,IFeatureMatcher)
        INSERT_FACTORY_ITEM(KnnMatcher    ,IFeatureMatcher)
    }

    IPoseSolverFactory::IPoseSolverFactory()
    {
        INSERT_FACTORY_ITEM(CVPoseSolver,IPoseSolver);
        INSERT_FACTORY_ITEM(ORBPoseSolver,IPoseSolver);
    }

    ITrajProcesserFactory::ITrajProcesserFactory()
    {
        INSERT_FACTORY_ITEM(PUniformVTrajProcesser   ,ITrajProcesser)
        INSERT_FACTORY_ITEM(PMultiVisionTrajProcesser,ITrajProcesser)
        INSERT_FACTORY_ITEM(PSfmVisonTrajProcesser   ,ITrajProcesser)
    }

    IOptimizerFactory::IOptimizerFactory()
    {
        INSERT_FACTORY_ITEM(G2oOptimizer,IOptimizer)
    }


#if USE_VIEW
#define INSERT_VIEWER_ITEM(F) { std::shared_ptr<VIEWERFACTORY> p(new F##Factory()); \
        mItems.insert(make_pair(p->name(),p));}
    IViewerFactory::IViewerFactory()
    {
        INSERT_VIEWER_ITEM(Position::PangolinViewer);
    }
#endif
}