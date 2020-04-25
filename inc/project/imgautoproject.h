/**
 *   imgautoproject.h 图像自动化工程
 *   
 *   add by tu li gen   2020.4.25
 * 
 */
#ifndef _IMGAUTOPROJECT_H_H_
#define _IMGAUTOPROJECT_H_H_

#include "P_Config.h"
#include "P_Data.h"


namespace Position
{
    //图像自动化配置文件
    class ImgAutoConfig : public PConfig
    {
    public:
        ImgAutoConfig(const std::string &path);
        ~ImgAutoConfig(){}
        //其他信息加载
        virtual void loadmore() ;
    protected:
        Position::StringConfigParam PrjPath;        //工程目录
        Position::StringConfigParam CamMatrixPath;  //相机参数文件路径
    };

    //图像自动化 数据接口
    class ImgAutoData : public PData
    {
    public:
        ImgAutoData(const std::shared_ptr<IConfig> &pcfg):PData(pcfg)
        {}
        //预处理数据
        virtual bool loadDatas();
    protected:
        //加载相机参数
        void loadCameraParams(const std::string &path);
        //加载track json
        bool loadTrackJson(const std::string &path,FrameDataVector &framedatas);
};
}

#endif