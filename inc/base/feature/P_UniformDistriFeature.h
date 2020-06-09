#ifndef _UNIFORMDISTRIFEATURE_H_H
#define _UNIFORMDISTRIFEATURE_H_H

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

        //返回sigma参数(主要用于优化 信息矩阵)
        virtual const FloatVector &getSigma2() const
        {
            return mSigmaVector;
        }
    protected:
        void detect(const Mat &img, KeyPtVector &keypts);

        void compute(const Mat &img, KeyPtVector &keypts, Mat &des);

        void createQuadTree(KeyPtVector &keypts);

        KeyPtVector distributeQuadTree(const KeyPtVector &vToDistributeKeys, const int &minX,
                                    const int &maxX, const int &minY, const int &maxY, const int &N);

    protected:
        cv::Ptr<cv::Feature2D> mFeature;
        Position::FloatVector mSigmaVector;
        int mMaxFeatures;
    };
    DECLAREIFACTORY(IFeature, UniformDistriFeature,Uniform)
} // namespace Position

#endif
