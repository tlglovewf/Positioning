#include "P_Controller.h"
#include "P_Factory.h"
#include "P_Map.h"
#include "P_Writer.h"


//间隔最大的帧数
const int maxThFrameCnt = 3;

//设置数据解析类型
PositionController::PositionController(const shared_ptr<Position::IDetector> &pdetecter, 
                                       const shared_ptr<Position::IData> &pdata,
                                       const shared_ptr<Position::IConfig> &pcfg):
                                        mpConfig(pcfg),mpData(pdata),mpDetector(pdetecter)
{
    assert(pdata);
    assert(pcfg);
    assert(pdetecter);

    bool bol = mpData->loadDatas() && mpDetector->init();
    if(!bol)
    {
        PROMT_S("PositionController Initialize failed!!!");
        PROMT_S("Please check config params!!!");
        exit(-1);
    }

    mpTrajProSelector = std::unique_ptr<Position::TrajProSelector>(new Position::TrajProSelector(pcfg,pdata));

    if(GETCFGVALUE(mpConfig,ViewEnable,int))
    {
        mpViewer = std::shared_ptr<Position::IViewer>(Position::PFactory::CreateViewer(Position::eVPangolin,pcfg));
    }
    mpChecker  = std::unique_ptr<Position::IChecker>(Position::PFactory::CreateChecker(Position::eNormalChecker));

    const Position::CameraParam &cam = mpData->getCamera();
    mpMulPositioner     = std::unique_ptr<Position::IPositioning>(Position::PFactory::CreatePositioning(Position::ePMultiImage,cam)); 
    mpSinglePositioner  = std::unique_ptr<Position::IPositioning>(Position::PFactory::CreatePositioning(Position::ePSingleImage,cam));
}

//运行
void PositionController::run()
{
    Position::FrameDataVIter it = mpData->begin();
    Position::FrameDataVIter ed = mpData->end();
    const std::string imgpath = GETCFGVALUE(mpConfig,ImgPath ,string) + "/";
    bool hasTarget =  false;
     while( it != ed)
     {//帧循环 构建局部场景
         int  oThs = 0;
         Position::FrameDataVector framedatas; 
         for(;it != ed ;++it)
         {//遍历帧
            
             const std::string picpath = imgpath + it->_name;
             it->_img = imread(picpath,IMREAD_UNCHANGED);
             it->_targets = mpDetector->detect(it->_img);

              if(!mpChecker->check(*it))
                 continue;

             framedatas.push_back(*it);
            //  if(it->_targets.empty())
            //  {
            //      hasTarget = false;

            //      if(oThs++ > maxThFrameCnt)
            //      {//连续帧没有探测到物体 闭合定位场景
            //          break;
            //      }

            //  }
            //  else
            //  {   
            //      oThs = 0;
            //      hasTarget = true;
            //  }
         }

         if(framedatas.empty())
         {
             continue;
         }
         //处理场景帧,计算位姿
         if(mpTrajProSelector->handle(framedatas))
         {
             PROMT_S("Traj have been processed suceesfully!");
             const std::shared_ptr<Position::IMap> &map = mpTrajProSelector->getMap();
             if(mpViewer)
             {
                 mpTrajProSelector->setViewer(mpViewer);
             }
             if(framedatas.size() < 2)
             {//如果只有一帧,使用单张直接定位
                //mpSinglePositioner->position(map->getAllFrames()[0]);
                PROMT_S("Single image position.");
             }
             else
             {//如果有多帧 使用多帧定位
                // mpMulPositioner->position(map);
                PROMT_S("Multi images position.");
             }
         }
         else
         {//位姿推算失败
            PROMT_S("------------------");
            PROMT_S("position failed.");
            PROMT_S("------------------");
            //ADD More
         }

         //完成一段轨迹推算  记录结果
         saveResult();
         mpTrajProSelector->reset();//重置状态
     }
     if(mpViewer)
     {//如果有可视接口 显示该段
         mpViewer->renderLoop();
     }
}


void PositionController::saveResult()
{
    //add more
}

