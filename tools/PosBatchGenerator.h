/**
 *   PosBatchGenerator.h
 *   
 *   add by tu li gen   2020.4.26
 * 
 */

#ifndef _POSBATCHGENERATOR_H_H_
#define _POSBATCHGENERATOR_H_H_
#include "P_Interface.h"

using namespace Position;


//定位 段生成器
class PosBatchGenerator
{
public:
    PosBatchGenerator(const std::shared_ptr<IData> &pdata):mpData(pdata)
    {}
    //加载轨迹文件文件
    virtual bool loadTraceFile(const std::string &path);


protected:
    std::shared_ptr<IData> mpData;
};


#endif