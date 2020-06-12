/**
 *   P_FeatureExtend.h
 *   
 *   add by tu li gen   2020.2.13
 * 
 */
#ifndef _FEATUREEXTEND_H_H
#define _FEATUREEXTEND_H_H

#include "P_Feature.h"

namespace Position
{
    //均匀分布特征  分块提取 + 四叉树过滤
    class UniformDistriFeature : public Position::PFeature
    {

    public:
        UniformDistriFeature();
        UniformDistriFeature(int nFeatures);
        ~UniformDistriFeature() {}
        //计算特征点
        virtual bool detect(const FrameData &frame, FeatureInfo &info);

    protected:
        void detect(const Mat &img, KeyPtVector &keypts);

        void compute(const Mat &img, KeyPtVector &keypts, Mat &des);

        KeyPtVector distributeQuadTree(const KeyPtVector &vToDistributeKeys, const int &minX,
                                    const int &maxX, const int &minY, const int &maxY, const int &N);

    protected:
        //! 重载实现 具体特征类实现
        virtual void featureInit(int featurecnt) = 0;

    protected:
        cv::Ptr<cv::Feature2D> mFeature;
        int mMaxFeatures;
    };
    //! sift 扩展
    class SiftFeatureExtend : public UniformDistriFeature
    {
    public:
        SiftFeatureExtend():UniformDistriFeature(){}
        SiftFeatureExtend(int nFeatures):UniformDistriFeature(nFeatures){}
    protected:
        //! 重载实现 具体特征类实现
        virtual void featureInit(int featurecnt);
    };

    DECLAREIFACTORY(IFeature, SiftFeatureExtend,SiftEx)
} // namespace Position

#endif
