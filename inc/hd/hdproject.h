#ifndef _HDPROJECT_H_H_
#define _HDPROJECT_H_H_

#include "P_Config.h"
#include "P_Data.h"

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
};

class HdData : public Position::PData
{
public:
    HdData(const std::shared_ptr<Position::IConfig> &pcfg);

     //预处理数据
    virtual bool loadDatas();
};

#endif