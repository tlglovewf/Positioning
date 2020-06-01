#include "P_MapDisplay.h"
#include "P_Factory.h"
#include "P_Map.h"
#include "P_IOHelper.h"
#include "P_Utils.h"
#include "P_GpsFusion.h"

//间隔最大的帧数
const int maxThFrameCnt = 3;


PMapDisplay::PMapDisplay(const shared_ptr<Position::IConfig> &pcfg, const shared_ptr<Position::IMap> &pmap)
{
#ifdef USE_VIEW

    mpViewer = std::shared_ptr<Position::IViewer>(Position::PFactory::CreateViewer(Position::eVPangolin, pcfg));

    mpViewer->setMap(pmap);

    mpViewer->renderLoop();

#endif
}

//设置数据解析类型
PMapDisplay::PMapDisplay(const shared_ptr<Position::IData> &pdata,
                         const shared_ptr<Position::IConfig> &pcfg,
                         int  eTjtype /*= 1*/,
                         bool useThread /*= true*/) :mbUseThread(useThread),mpConfig(pcfg), mpData(pdata)
{
    assert(pdata);
    assert(pcfg);

    bool bol = mpData->loadDatas();
    if (!bol)
    {
        // PROMT_S("PositionController Initialize failed!!!");
        // PROMT_S("Please check config params!!!");
        LOG_CRIT("No Datas!!!!");
        exit(-1);
    }

    mpTrajProSelector = std::unique_ptr<Position::PoseEstimator>(new Position::PoseEstimator(pcfg, pdata->getCamera(),eTjtype));

#ifdef USE_VIEW
    if (GETCFGVALUE(mpConfig, ViewEnable, int))
    {
        mpViewer = std::shared_ptr<Position::IViewer>(Position::PFactory::CreateViewer(Position::eVPangolin, pcfg));
        mpTrajProSelector->setViewer(mpViewer);
        if(mbUseThread)
            mptViewer = std::unique_ptr<std::thread>(new thread(&Position::IViewer::renderLoop,mpViewer));
    }
    else
    {
        PROMT_S("Visualization is closed!!!");
    }
    
#endif

    mpGpsFunsion = std::unique_ptr<Position::IGpsFusion>(new Position::GpsFunsion());
}

//运行
void PMapDisplay::run()
{
    int stNo = GETCFGVALUE(mpConfig,StNo,int);
    int edNo = max(stNo, GETCFGVALUE(mpConfig,EdNo,int));
    LOG_INFO_F(">>>>>>>>>> From %d to %d !!! <<<<<<<<<<",stNo,edNo);
    Position::FrameDataPtrVIter it = mpData->begin() + stNo;
    Position::FrameDataPtrVIter ed = mpData->begin() + edNo;
    if(ed > mpData->end() || stNo == edNo)
    {
        ed = mpData->end();
    }
    Position::FrameDataPtrVector tempDatas(it,ed);

 
    const std::string imgpath = GETCFGVALUE(mpConfig, ImgPath, string) + "/";

    if(!mpTrajProSelector->process(tempDatas,imgpath))
    {
        LOG_ERROR("Trace Handle Error.");
        return ;
    }

    mpGpsFunsion->fuse(mpTrajProSelector->getMap(), mpData->getCamera());
#ifdef USE_VIEW
    if (mpViewer)
    { //如果有可视接口 显示该段
        if(mptViewer)
        {
            if(mptViewer && mptViewer->joinable())
            {
                mptViewer->join();
            }
        }
        else
        {
            mpViewer->renderLoop();
        }
    }
#endif
    //完成一段轨迹推算  记录结果
    saveResult();
    mpTrajProSelector->reset(); //重置状态

    mpTrajProSelector->release();
}

void PMapDisplay::saveResult()
{
    LOG_INFO_F("Save Result %d",GETCFGVALUE(mpConfig, MapSave, int))
    if (GETCFGVALUE(mpConfig, MapSave, int))
    {
        SETMAPSERIALIZATIONMAP(mpTrajProSelector->getMap());

        const std::string path = GETCFGVALUE(mpConfig, OutPath, string) + "/";

        SAVEMAPFRAME(path + "frames.txt");
        SAVEMAPFRAME(path + "points.txt");

        LOG_INFO("Save Result Finished.");
    }
}
