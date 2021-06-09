#ifndef _WEIYAPROJECT_H_H_
#define _WEIYAPROJECT_H_H_

#include "P_Config.h"
#include "P_Data.h"

//维亚配置数据
class WeiyaConfig : public Position::PConfig
{
public:
    WeiyaConfig(const std::string &path);
protected:
    //其他信息加载
    virtual void loadmore();
protected:
    Position::StringConfigParam ExtriPath;  //相机参数文件路径
    Position::StringConfigParam BsPath;     //安置参数文件路径
};

//维亚数据处理
class WeiyaData : public Position::PFrameData
{
public:
    WeiyaData(const std::shared_ptr<Position::IConfig> &pcfg);
    //预处理数据
    virtual bool loadDatas();
    //根据图像名取时间(天秒)
    virtual double getTimeFromName(const std::string &name);
};

#endif