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

    //自动化 batch 列表类
    class ImgAutoPrjList : public ProjList
    {
    public:
        //信息索引 图片索引，目标索引
        typedef std::pair<int,int> InfoIndex;
        struct TrackerItem
        {
            int       id;           //目标id
            InfoIndex stno;         //第一帧出现的索引
            InfoIndex edno;         //最后一帧出现的索引
            int       maxsize;      //出现的帧数
        };

        typedef std::vector<TrackerItem>        TrackerItemVector;
        typedef TrackerItemVector::iterator     TrackerItemVIter;

         //加载项目列表
        virtual void loadPrjList(const std::string &path);
        //加载地图
        virtual void loadMap(const std::string &path);
        //保存地图
        virtual void saveMap(const std::string &path);

    protected:
        //接写tracker 行
         void parseTracker(const std::string &line);

    protected:
        StringVector      mTrkLines;
        TrackerItemVector mTrackerInfos;
    };
    
}

#endif