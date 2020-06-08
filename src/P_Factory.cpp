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
     /*
     * 创建对象
     */
    IBlockMatcher* PFactory::CreateBlockMatcher(eBlockMatcherType type,const Mat &img, const Point2f &pt)
    {
        switch (type) {
            case eBMNCC:
                return new NCC_BlockMatcher(img,pt);
                //add more
            default:
                return NULL;
        }
    }

     /*
      * 优化
      */
     IOptimizer* PFactory::CreateOptimizer(eOptimizerType type)
     {
         switch(type)
         {
             case eOpG2o:
                return new G2oOptimizer();
             default:
                {
                    assert(NULL);
                    return NULL;
                }
         }
     }

#ifdef USE_VIEW
    /*
     * 创建可视化
     */
    IViewer* PFactory::CreateViewer(eViewerType type,const std::shared_ptr<IConfig> &pcfg)
    {
        switch(type)
        {
            case eVPangolin:
                return new Pangolin_Viewer(pcfg);
            default:
                {
                    assert(NULL);
                    return NULL;
                }
        }
    }
#endif


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
        std::unique_ptr<Position::IViewer> pv(Position::PFactory::CreateViewer(Position::eVPangolin,pCfg));
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

#define INSERT_FEATURE_ITEM(F) { std::shared_ptr<FEATUREFACTORY> p(new F##Factory()); \
        mItems.insert(make_pair(p->name(),p));}

    IFeatureFactory::IFeatureFactory()
    {
        INSERT_FEATURE_ITEM(Position::ORBFeature)
        INSERT_FEATURE_ITEM(Position::SiftFeature)
        INSERT_FEATURE_ITEM(Position::UniformDistriFeature)
    }
#undef INSERT_FEATURE_ITEM

#define INSERT_MATCHER_ITEM(F) { std::shared_ptr<FEATUREMATCHERFACTORY> p(new F##Factory()); \
        mItems.insert(make_pair(p->name(),p));}
    IFeatureMatcherFactory::IFeatureMatcherFactory()
    {
        INSERT_MATCHER_ITEM(Position::HanMingMatcher)
        INSERT_MATCHER_ITEM(Position::KnnMatcher    )
    }
#undef INSERT_MATCHER_ITEM

#define INSERT_POSESOLVER_ITEM(F) { std::shared_ptr<POSESOLVERFACTORY> p(new F##Factory()); \
        mItems.insert(make_pair(p->name(),p));}
    IPoseSolverFactory::IPoseSolverFactory()
    {
        INSERT_POSESOLVER_ITEM(Position::CVPoseSolver);
        INSERT_POSESOLVER_ITEM(Position::ORBPoseSolver);
    }
#undef INSERT_POSESOLVER_ITEM

#define INSERT_TRJPROCESSER_ITEM(F) { std::shared_ptr<TRAJPROCESSERFACTORY> p(new F##Factory()); \
        mItems.insert(make_pair(p->name(),p));}
    ITrajProcesserFactory::ITrajProcesserFactory()
    {
        INSERT_TRJPROCESSER_ITEM(Position::PUniformVTrajProcesser   )
        INSERT_TRJPROCESSER_ITEM(Position::PMultiVisionTrajProcesser)
        INSERT_TRJPROCESSER_ITEM(Position::PSfmVisonTrajProcesser   )
    }
#undef INSERT_TRJPROCESSER_ITEM
}