/**
 *   P_Controller.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PCONTROLLER_H_H_
#define __PCONTROLLER_H_H_
#include "P_Interface.h"

// position controller class
class PositionController
{
public:
    enum DataParseType{eWeiyaType};
    enum ImgDetectType{eSSDType};
    //构造函数
    PositionController(const string &cfgpath,DataParseType datatype,ImgDetectType imgtype = eSSDType);

    //初始化
    bool init(); 

    

protected:

    

protected:
    std::shared_ptr<Position::IConfig>    mpConfig;
    std::unique_ptr<Position::IData>      mpData;
    std::unique_ptr<Position::IDetector>  mpDetector;
    std::unique_ptr<Position::IViewer>    mpViewer;
};

#endif