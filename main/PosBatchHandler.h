/**
 *   PosBatchHandler.h
 *   
 *   add by tu li gen   2020.4.26
 * 
 */

#ifndef _POSBATCHGENERATOR_H_H_
#define _POSBATCHGENERATOR_H_H_
#include "P_Interface.h"
#include "P_PoseEstimator.h"
#include "project/imgautoproject.h"

using namespace Position;



//! 定位处理器
class PosBatchHandler
{
public:
    PosBatchHandler(const std::shared_ptr<IConfig>           &pcfg      ,
                    const std::shared_ptr<IProjList>         &prjlist   ,
                    const std::shared_ptr<IVisualPositioner> &positioner,
                    const CameraParam   &cam);

    //! 加载识别文件的信息,并根据识别物体进行图片分批
    bool loadTrackerInfos(const std::string &path);
  
    //! 保存batches
    void saveBatchPose(const std::string &path)
    {
        //add more .
    }

    //! 保存结果
    void saveResult(const std::string &path);
    
    //! 保存位姿
    void savePose(const std::string &path);

    //! 估算batch 位姿
    void poseEstimate();

    //! 目标定位
    void targetPositioning();

protected:
    std::shared_ptr<IConfig>            mpConfig;
    std::shared_ptr<IProjList>          mPrjList;
    std::shared_ptr<IVisualPositioner>  mPositioner;
    std::shared_ptr<IGpsFusion>         mGpsFusion;

    CameraParam                         mCamera;
    PoseEstimator                       mPoseEstimator;
};


#endif