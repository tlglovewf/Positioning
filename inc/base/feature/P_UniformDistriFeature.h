#ifndef  _PUNIFORMDISTRIFEATURE_H_H
#define  _PUNIFORMDISTRIFEATURE_H_H

#include "P_Feature.h"

using namespace Position;

//均匀分布特征  分块提取 + 四叉树过滤
class PUniformDistriFeature : public Position::PFeature
{

public:
    PUniformDistriFeature(int nFeatures);
    ~PUniformDistriFeature(){}
    //计算特征点
    virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript);

    //返回sigma参数(主要用于优化 信息矩阵)
    virtual const FloatVector& getSigma2() const 
    {
        return mSigmaVector;
    }
    //获取名称
    virtual std::string name()const 
    {
        return _TOSTRING(Uniform);
    }
protected:
    void detect(const Mat &img, KeyPtVector &keypts);

    void compute(const Mat &img, KeyPtVector &keypts, Mat &des);

    void createQuadTree(KeyPtVector &keypts);

    KeyPtVector distributeQuadTree(const KeyPtVector& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N);
protected:
    cv::Ptr<cv::Feature2D>  mFeature;
    Position::FloatVector   mSigmaVector;
    int                     mMaxFeatures;
};

#endif
