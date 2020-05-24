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
#include "P_ProjList.h"
#include "P_Positioner.h"

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
        bool loadTrackJson(const std::string &path,FrameDataPtrVector &framedatas);    
        //解析每个文件对应的跟踪文件
        void ParseTrkTxt(const string &txtfile, StringVector &lines);
    };

    //自动化 batch 列表类
    class ImgAutoPrjList : public ProjList
    {
    public:
        ImgAutoPrjList(const std::shared_ptr<IData> &pdata):
        mpData(pdata)
        {}
         //加载项目列表
        virtual void loadPrjList(const std::string &path);
        //加载地图
        virtual void load(const std::string &path);
        //保存地图
        virtual void save(const std::string &path);
    protected:
        //解析tracker.txt
        void parseTracker(const std::string &line);
    protected:
        StringVector                        mTrkLines;
        std::shared_ptr<IData>              mpData;
    };
}

#endif