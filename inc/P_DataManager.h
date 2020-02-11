/**
 *   P_DataManager.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PDATAMANAGER_H_H_
#define __PDATAMANAGER_H_H_
#include "P_Interface.h"
#include "P_Writer.h"
// position controller class
class PDataManager
{
public:
    enum DataParseType{eWeiyaType};

    //单例接口
    static PDataManager* getSingleton()
    {
        static PDataManager singleton;
        return &singleton;
    }

    //设置数据解析类型
    void setDataParseType(DataParseType datatype)
    {
        PROMT_V("data parse type is ", datatype);
        

    }

protected:

    //disable construct and copy function
    PDataManager();
    PDataManager(const PDataManager &);
};

#endif