#include "P_MapDisplay.h"
#include "P_Factory.h"
#include "P_Map.h"
#include "P_Writer.h"
#include "P_Utils.h"
#include "P_GpsFusion.h"

//间隔最大的帧数
const int maxThFrameCnt = 3;

//设置数据解析类型
PMapDisplay::PMapDisplay(const shared_ptr<Position::IData> &pdata,
                                       const shared_ptr<Position::IConfig> &pcfg):
                                        mpConfig(pcfg),mpData(pdata)
{
    assert(pdata);
    assert(pcfg);

    bool bol = mpData->loadDatas();
    if(!bol)
    {
        PROMT_S("PositionController Initialize failed!!!");
        PROMT_S("Please check config params!!!");
        exit(-1);
    }

    mpTrajProSelector = std::unique_ptr<Position::TrajProSelector>(new Position::TrajProSelector(pcfg,pdata));

#ifdef USE_VIEW
    if(GETCFGVALUE(mpConfig,ViewEnable,int))
    {
        mpViewer = std::shared_ptr<Position::IViewer>(Position::PFactory::CreateViewer(Position::eVPangolin,pcfg));
        mpTrajProSelector->setViewer(mpViewer);
        //mptViewer = std::unique_ptr<std::thread>(new thread(&Position::IViewer::renderLoop,mpViewer));
    }
#endif

    mpGpsFunsion  = std::unique_ptr<Position::IGpsFusion>(new Position::GpsFunsion());
}

//运行
void PMapDisplay::run()
{
    Position::FrameDataVIter it = mpData->begin();
    Position::FrameDataVIter ed = mpData->end();
    const std::string imgpath = GETCFGVALUE(mpConfig,ImgPath ,string) + "/";

    //帧循环 构建局部场景
    for(;it != ed ;++it)
    {//遍历帧
       
        const std::string picpath = imgpath + it->_name;
        it->_img = imread(picpath,IMREAD_UNCHANGED);
        mpTrajProSelector->handle(&(*it));
        (*it)._img.release();
    }

    //等待线程处理
    mpTrajProSelector->waitingForHandle();

    mpGpsFunsion->fuse(mpTrajProSelector->getMap(),mpData->getCamera());
#ifdef USE_VIEW
    if(mpViewer)
    {//如果有可视接口 显示该段
        mpViewer->renderLoop();
        // if(mptViewer && mptViewer->joinable())
        // {
        //     mptViewer->join();
        // }
    }
#endif
    //完成一段轨迹推算  记录结果
    saveResult();
    mpTrajProSelector->reset();//重置状态

    mpTrajProSelector->release();
}


void PMapDisplay::saveResult()
{
    if(GETCFGVALUE(mpConfig,MapSave,int))
    {
        Position::Time_Interval time;
        time.start();
        Position::MapSerManager::Instance()->setMap(mpTrajProSelector->getMap());
        PROMT_S("Begin to save map");
        const std::string path = GETCFGVALUE(mpConfig,OutPath,string) + "/";
        Position::MapSerManager::Instance()->tracSerPtr()->saveMap(path + "trac.txt");
        time.prompt("Saving trace cost:");
        Position::MapSerManager::Instance()->mpPtSerPtr()->saveMap(path + "mpts.txt");
        time.prompt("Saving map points cost:");
        PROMT_S("Save map successflly!");
    }
}

