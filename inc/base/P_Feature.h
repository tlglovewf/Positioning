/**
 *   P_Feature.h
 *   
 *   add by tu li gen   2020.2.13
 * 
 */
#ifndef __PFEATURE_H_H_
#define __PFEATURE_H_H_
#include "P_Interface.h"

namespace Position
{
    //特征提取
    class PFeature : public IFeature
    {
    public:
        //设置配置文件
        PFeature(const std::shared_ptr<IConfig> &pcfg):mCfg(pcfg)
        {
        }

        //计算特征点
        virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript)
        {
            assert(NULL);
            return false;
        }

         //返回sigma参数(主要用于优化 信息矩阵)
        virtual const FloatVector& getSigma2() const 
        {
            assert(NULL);
            return FloatVector();
        }
    protected:
        //初始化
        virtual void init() = 0;
    protected:
        std::shared_ptr<IConfig> mCfg;
    };

    //cv原库orb特征点
    class PCVORBFeature : public PFeature
    {
    public:
        PCVORBFeature(const std::shared_ptr<IConfig> &pcfg);
          //计算特征点
        virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript);

    protected:
         //初始化
        virtual void init();

    protected:
        cv::Ptr<cv::Feature2D>  mpFeature;
    };
}

#endif