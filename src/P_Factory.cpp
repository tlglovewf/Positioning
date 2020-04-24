#include "P_Factory.h"
#include "P_BlockMatcher.h"
#include "P_ORBFeature.h"
#include "P_PoseEstimation.h"
#include "P_Optimizer.h"
#include "P_Positioning.h"
#include "P_FeatureMatcher.h"
#include "P_Checker.h"

#include "P_PangolinViewer.h"
#include "P_UniformVTrajProcesser.h"
#include "P_MultiVisionTrajProcesser.h"

#include "P_Writer.h"

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
    * 特征点
    */
    IFeature* PFactory::CreateFeature(eFeatureType type,const std::shared_ptr<IConfig> &pcfg)
    {
        switch(type)
        {
            case eFeatureOrb:
                return new ORBFeature(pcfg);
            case eFeatureCVOrb:
                return new PCVORBFeature(pcfg);
            default:
                return NULL;
        }
    }

    /*
     * 创建位姿推算
     */
    IPoseEstimation* PFactory::CreatePoseEstimation(ePoseEstimationType type)
    {
        switch(type)
        {
            case ePoseEstOrb:
                return new ORBPoseEstimation();
            case ePoseEstCV:
                return new CVPoseEstimation();
            default:
                {
                    assert(NULL);
                    return NULL;
                }
                
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

     /*
      * 定位
      */
     IPositioning* PFactory::CreatePositioning(ePositioningType type,const CameraParam  &cam)
     {
         switch (type)
         {
            case ePSingleImage:
                return new SingleImgPositioning(cam);
            case ePMultiImage:
                return new MultiImgPositioning(cam);
            case ePDepthImage:
                /* code */
                return new DepthLImgPositioning(cam);
            default:
            {
                assert(NULL);
                return NULL;
            }
         }
     }
    /*
     * 创建匹配器
     */
     IFeatureMatcher* PFactory::CreateFeatureMatcher(eFeatureMatcherType type, float ratio, bool bcheckori /* =true */)
     {
         switch(type)
         {
             case eFMDefault:
                return new PFeatureMatcher(ratio,bcheckori);
             case eFMKnnMatch:
                return new PKnnMatcher();
            default:
                {
                    assert(NULL);
                    return NULL;
                }
         }
     }
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

    /*
     * 创建跟踪对象
     */
    ITrajProcesser* PFactory::CreateTrajProcesser(eTrajProcesserType type, 
                                                  const std::shared_ptr<IConfig> &pcfg,
                                                  const std::shared_ptr<IData> &pdata)
    {
        switch(type)
        {
            case eTjUniformSpeed:
                return new PUniformVTrajProcesser(pcfg,pdata);
            case eTjMultiVision:
                return new PMultiVisionTrajProcesser(pcfg,pdata);
            default:
                return new PTrajProcesser();
        }
    }

    /*
     * 优化
     */
    IChecker* PFactory::CreateChecker(eCheckerType type)
    {
        switch (type)
        {
            case eNormalChecker:
                return new PChecker();
            default:
                {
                    assert(NULL);
                    return NULL;
                }
        }   
    }


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
        //可视化帧数据
        std::unique_ptr<Position::IViewer> pv(Position::PFactory::CreateViewer(Position::eVPangolin,pCfg));
        std::shared_ptr<Position::IMap> pmap(new Position::PMap());
        setMap(pmap);
        mpTracSer->loadMap(trac);
        mpPtSer->loadMap(mpts);
        pv->setMap(pmap);
        pv->renderLoop();
    }
    //融合地图  secMap -> baseMap
    void MapSerManager::combineMap(std::shared_ptr<IMap> &baseMap, const std::shared_ptr<IMap> &secMap)
    {
        KeyFrameVector fms  = baseMap->getAllFrames();
        KeyFrameVector sfms = secMap->getAllFrames();

        IMap::SortFrames(fms);

        //获取第一个地图最后一帧 当做第二个地图的起止位姿
        IKeyFrame *pLast = *fms.rbegin(); 

        for(size_t i = 0;i < sfms.size(); ++i)
        {
            IFrame *frame = new PFrame(sfms[i]->getData(),baseMap->frameCount());
            frame->setPose(sfms[i]->getPose() * pLast->getPose());
            baseMap->createKeyFrame(frame);
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
}