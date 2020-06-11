#include "PosBatchHandler.h"
#include "P_IOHelper.h"
#include "project/imgautoproject.h"
#include "P_Checker.h"
#include "P_Factory.h"
#include "P_Utils.h"

#if USE_VIEW
std::shared_ptr<Position::IViewer> s_Viewer;
#endif


PosBatchHandler::PosBatchHandler(const std::shared_ptr<IConfig>           &pcfg      ,
                                 const std::shared_ptr<IProjList>         &prjlist   ,
                                 const std::shared_ptr<IVisualPositioner> &positioner,
                                 const CameraParam                        &cam):
mpConfig(pcfg),
mPrjList(prjlist),
mPositioner(positioner),
mPoseEstimator(pcfg,cam,0),
mCamera(cam)
{
     assert(pcfg);
     assert(prjlist);
     assert(positioner);

#if USE_VIEW
s_Viewer = std::shared_ptr<Position::IViewer>(GETVIEWER());
s_Viewer->setMap(mPoseEstimator.getMap());
#endif

}

//加载轨迹文件文件
bool PosBatchHandler::loadTrackerInfos(const std::string &path)
{
    if(!PATHCHECK(path))
        return false;
    assert(mPrjList);

    mPrjList->loadPrjList(path);
    
    return !mPrjList->getPrjList().empty();
}

void PosBatchHandler::saveResult(const std::string &path)
{
    LOG_INFO("Save result ...");
    mPrjList->save(path);
    LOG_INFO_F("Result have been saved to %s",path.c_str());
}


void PosBatchHandler::savePose(const std::string &path)
{
    try
    {
        LOG_INFO_F("Begin To Save Pose :%s",path.c_str());

        Position::PrjWRHelper::Instance()->setPrjList(mPrjList);

        Position::PrjWRHelper::Instance()->writePrjResult(path);

        LOG_INFO("Pose Saved Successfully.");
    }
    catch(const std::exception& e)
    {
        LOG_ERROR_F("Save Pose Error : %s",e.what());
    }
}

//估算batch 位姿
void PosBatchHandler::poseEstimate()
{
    TrackerItemVector &targets = mPrjList->trackInfos();
    const std::string imgpath  = GETCFGVALUE(mpConfig,ImgPath,string);
    if(targets.empty())
    {
        LOG_WARNING("Target List Empty.");
        return;
    }
       

    LOG_INFO("PoseEstimate ...");

    for(size_t i = 1; i < targets.size(); ++i)
    {
       if(NULL == targets[i].batch)
       {
           LOG_WARNING("Target Batch Empty.");
           return;
       }

       LOG_INFO_F("Batch : %d - %d PoseEst",targets[i].id,targets[i].batch->_fmsdata.size());
    
       //先赋值为最后一帧出现的位置
       targets[i].blh = (*targets[i].batch->_fmsdata.rbegin())->_pos.pos;
       if(targets[i].batch->_fmsdata.size() >= 2)
       {//仅在位姿估算成功后,进行量测定位 
           if(mPoseEstimator.process(targets[i].batch->_fmsdata,imgpath))
           {//位姿估算成功,对batch中每个位姿进行赋值
#if USE_VIEW
            cout << "DISPLAY ...." << endl;
            s_Viewer->renderLoop();
#endif
               KeyFrameVector frames = mPoseEstimator.getMap()->getAllFrames();
               cout << "FmSize:" << frames.size() << " " << targets[i].batch->_fmsdata.size() << endl;
               assert(frames.size() == targets[i].batch->_fmsdata.size());
               for(size_t m = 0; m < frames.size(); ++m)
               {
                   assert(targets[i].batch->_fmsdata[m]->_name == frames[m]->getData()->_name);
                   targets[i].batch->_fmsdata[m] = frames[m]->getData();
                   targets[i].batch->_poses.emplace_back(frames[m]->getPose());
               }
           }
       }
       LOG_INFO_F("Batch : %d PoseEst Finished..",targets[i].id);
       mPoseEstimator.reset();
    }
    LOG_INFO("PoseEstimate Finished.");
}


//目标定位
void PosBatchHandler::targetPositioning()
{
    TrackerItemVector &targets = mPrjList->trackInfos();
    LOG_INFO("Targets Positioning ...")
    for(size_t i = 0; i < targets.size(); ++i)
    {
        TrackerItem &target = targets[i];
        if(mPositioner->position(target))
        {
            LOG_WARNING_F("Target %d Calc Pos Failed",target.id);
        }         
    }
    LOG_INFO("Target Positioning Finished.");
}