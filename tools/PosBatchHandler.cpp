#include "PosBatchHandler.h"
#include "P_Writer.h"
#include "project/imgautoproject.h"
#include "P_Checker.h"
#include "P_Factory.h"
#include "P_Utils.h"


PosBatchHandler::PosBatchHandler(const std::shared_ptr<IConfig>           &pcfg      ,
                                 const std::shared_ptr<IProjList>         &prjlist   ,
                                 const std::shared_ptr<IVisualPositioner> &positioner,
                                 const CameraParam                        &cam):
mpConfig(pcfg),
mPrjList(prjlist),
mPositioner(positioner),
mPoseEstimator(pcfg,cam),
mCamera(cam)
{
     assert(pcfg);
     assert(prjlist);
     assert(positioner);
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
const int test_count = 1;
//估算batch 位姿
void PosBatchHandler::poseEstimate()
{
    TrackerItemVector &targets = mPrjList->trackInfos();
    PrjBatchVector    &batches = mPrjList->getPrjList();
    const std::string imgpath  = GETCFGVALUE(mpConfig,ImgPath,string);
    if(targets.empty())
        return;
    assert(targets.size()  == batches.size());

    LOG_INFO("PoseEstimate ...");

    for(size_t i = 0; i < targets.size(); ++i)
    {
    //    if(i >= test_count)
    //        break;
       LOG_INFO_F("Batch : %d - %d PoseEst",targets[i].id,targets[i].batch->_fmsdata.size());
       assert(std::to_string(targets[i].id) == batches[i]->_btname);
       //先赋值为最后一帧出现的位置
       targets[i].blh = (*batches[i]->_fmsdata.rbegin())->_pos.pos;
       if(batches[i]->_fmsdata.size() >= 2)
       {//仅在位姿估算成功后,进行量测定位 
           if(mPoseEstimator.process(batches[i]->_fmsdata,imgpath))
           {//位姿估算成功,对batch中每个位姿进行赋值
               KeyFrameVector frames = mPoseEstimator.getMap()->getAllFrames();
              
               assert(frames.size() == batches[i]->_fmsdata.size());
               for(size_t m = 0; m < frames.size(); ++m)
               {
                   assert(batches[i]->_fmsdata[m]->_name == frames[m]->getData()->_name);
                   batches[i]->_fmsdata[m] = frames[m]->getData();
                   batches[i]->_poses.emplace_back(frames[m]->getPose());
               }
           }
       }
       LOG_INFO_F("Batch : %d PoseEst Finished..",targets[i].id);
       mPoseEstimator.reset();
    }
    LOG_INFO("PoseEstimate Finished.");
    // for(auto item : batches[0]->_poses)
    // {
    //     cout << item << endl;
    // }
}


//目标定位
void PosBatchHandler::targetPositioning()
{
    TrackerItemVector &targets = mPrjList->trackInfos();
    PrjBatchVector    &batches = mPrjList->getPrjList();
    assert(targets.size()  == batches.size());
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