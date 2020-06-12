
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
        POptimizer()
        {
            initInforSigma2();
        }
        //单张位姿优化
        virtual int frameOptimization(IKeyFrame *pFrame, const FloatVector &sigma2) 
        {
            assert(NULL);
            return 0;
        }
        
        //ba 优化
        virtual void bundleAdjustment(const KeyFrameVector &keyframes,const MapPtVector &mappts,int nIterations = 5,
                                        bool *pbStopFlag = NULL,int nIndex = 0,bool bRobust = true) 
                                        {
                                            assert(NULL);
                                        }

        //设置相机参数
        virtual void setCamera(const CameraParam &mCam)
        {
            assert(!mCam.K.empty());

            mFx = mCam.K.at<MATTYPE>(0,0);
            mFy = mCam.K.at<MATTYPE>(1,1);
            mCx = mCam.K.at<MATTYPE>(0,2);
            mCy = mCam.K.at<MATTYPE>(1,2);
        }

    protected:
        //! 根据金字塔层级和缩放比 计算层级对应的sigma参数 主要用于优化时的信息矩阵比例
        void initInforSigma2();
    protected:
        float mFx;
        float mFy;
        float mCx;
        float mCy;
        FloatVector               mSigma2;
    };

    //g2o 优化类
    class G2oOptimizer : public POptimizer
    {
    public:
        //单张位姿优化
        virtual int frameOptimization(IKeyFrame *pFrame);

        //ba 优化
        virtual void bundleAdjustment(const KeyFrameVector &keyframes,const MapPtVector &mappts,int nIterations = 5,
                                        bool *pbStopFlag = NULL,int nIndex = 0,bool bRobust = true);
    };

    DECLAREIFACTORY(IOptimizer, G2oOptimizer,G2o)
}

#endif