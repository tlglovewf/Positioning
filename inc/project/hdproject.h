#ifndef _HDPROJECT_H_H_
#define _HDPROJECT_H_H_

#include "P_Config.h"
#include "P_Data.h"
#include "P_ProjList.h"

class HdConfig : public Position::PConfig
{
 public:

    HdConfig(const std::string &path);
    ~HdConfig(){}
    //其他信息加载
    virtual void loadmore() ;

    protected:
        Position::FloatConfigParam    HdCamFx;
        Position::FloatConfigParam    HdCamFy;
        Position::FloatConfigParam    HdCamCx;
        Position::FloatConfigParam    HdCamCy;
        Position::FloatConfigParam    HdCamk1;
        Position::FloatConfigParam    HdCamk2;
        Position::FloatConfigParam    HdCamk3;
        Position::FloatConfigParam    HdCamp1;
        Position::FloatConfigParam    HdCamp2;

};

class HdData : public Position::PFrameData
{
public:
    HdData(const std::shared_ptr<Position::IConfig> &pcfg);

     //预处理数据
    virtual bool loadDatas();
};

// serialization interface
class HdPosePrj : public Position::ProjList
{
public:
    //加载项目列表
    virtual void loadPrjList(const std::string &path);
    //加载地图
    virtual void load(const std::string &path);
    //保存地图
    virtual void save(const std::string &path);
};


#endif