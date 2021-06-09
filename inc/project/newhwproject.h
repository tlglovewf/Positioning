/**
 *   newhwproject.h 图像自动化 原始测试数据
 *   
 *   add by tu li gen   2020.5.27
 * 
 */
#ifndef _NEWHWPROJECT_H_H_
#define _NEWHWPROJECT_H_H_

#include "project/imgautoproject.h"

class NewHwProjectData : public ImgAutoData
{
public:
    NewHwProjectData(const std::shared_ptr<Position::IConfig> &pcfg) : ImgAutoData(pcfg) {}
    //预处理数据
    virtual bool loadDatas();

protected:
    //加载相机参数
    virtual void loadCameraParams(const std::string &path);
    //加载trackjson
    bool loadTrackJson(const std::string &path, Position::FrameDataPtrVector &framedatas);

protected:
    CameraCfgDB mCameraDB;
};


#endif