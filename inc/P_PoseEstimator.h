/**
 *   P_PoseEstimator.h
 *   
 *   add by tu li gen   2020.2.14
 * 
 */
#ifndef __POSEESTIMATOR_H_
#define __POSEESTIMATOR_H_
#include "P_Interface.h"

namespace Position
{
    //轨迹处理选择类
    class PoseEstimator 
    {
    public:
        //构造函数
        PoseEstimator(const std::shared_ptr<IConfig> &pCfg, const CameraParam &cam,int eType = 1);

        //处理帧数据
        bool process(const FrameDataPtrVector &datas,const std::string &imgpath = "");

        //处理单帧
        bool handle( FrameData *data);

        //获取地图
        const std::shared_ptr<IMap>& getMap();

        //设置可视接口
        void setViewer(const std::shared_ptr<IViewer> &pviewer);

        //重置状态
        void reset();

        //等待处理
        void waitingForHandle();

        //释放
        void release()
        {
            if(mpCurrentTrajPro)
            {
                mpCurrentTrajPro->over();
            }
        }

    protected:
        std::shared_ptr<ITrajProcesser>   mpCurrentTrajPro;  

        std::shared_ptr<ITrajProcesser>   mpUniformVTrajPro;//匀速运动模型

        std::shared_ptr<ITrajProcesser>   mpSimpleTrajPro; //简单模型

        std::shared_ptr<IViewer>          mpViewer;//可视化接口

        //add more
    };
} // namespace Position





#endif