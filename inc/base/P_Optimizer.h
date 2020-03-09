
/**
 *   P_Optimizer.h
 *   
 *   add by tu li gen   2020.2.19
 * 
 */
#ifndef __POPTIMIZER_H_H_
#define __POPTIMIZER_H_H_
#include "P_Interface.h"
#include "P_Map.h"

namespace Position
{
    //优化基类
    class POptimizer : public IOptimizer
    {
    public:
        //单张位姿优化
        virtual int frameOptimization(IKeyFrame *pFrame, const FloatVector &sigma2) 
        {
            assert(NULL);
        }
        
        //ba 优化
        virtual void bundleAdjustment(const KeyFrameVector &keyframes,const MapPtVector &mappts, const FloatVector &sigma2,int nIterations = 5,
                                        bool *pbStopFlag = NULL,int nIndex = 0,bool bRobust = true) 
                                        {
                                            assert(NULL);
                                        }

        //设置相机参数
        virtual void setCamera(const CameraParam &mCam)
        {
            assert(!mCam.K.empty());

            mFx = mCam.K.at<double>(0,0);
            mFy = mCam.K.at<double>(1,1);
            mCx = mCam.K.at<double>(0,2);
            mCy = mCam.K.at<double>(1,2);
        }
        //设置特征提取类
        virtual void setFeature(const std::shared_ptr<IFeature> &feature)
        {
            mpFeature = feature;
        }
    protected:
        float mFx;
        float mFy;
        float mCx;
        float mCy;
        std::shared_ptr<IFeature> mpFeature;
    };

    //g2o 优化类
    class G2oOptimizer : public POptimizer
    {
    public:
        //单张位姿优化
        virtual int frameOptimization(IKeyFrame *pFrame, const FloatVector &sigma2);

        //ba 优化
        virtual void bundleAdjustment(const KeyFrameVector &keyframes,const MapPtVector &mappts, const FloatVector &sigma2,int nIterations = 5,
                                        bool *pbStopFlag = NULL,int nIndex = 0,bool bRobust = true);
    };

}

#endif