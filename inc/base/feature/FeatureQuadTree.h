#ifndef  _FEATUREQUADTREE_H_H
#define  _FEATUREQUADTREE_H_H

#include "P_Interface.h"
#include <opencv2/xfeatures2d.hpp>

using namespace Position;

//特征四叉树
class FeatureQuadTree : public Position::IFeature
{

public:
    
    FeatureQuadTree();
    ~FeatureQuadTree(){}
    //计算特征点
    virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript);

    void detect(const Mat &img, KeyPtVector &keypts);

    void compute(const Mat &img, KeyPtVector &keypts, Mat &des);

    //返回sigma参数(主要用于优化 信息矩阵)
    virtual const FloatVector& getSigma2() const 
    {
        return mSigmaVector;
    }
protected:

    void createQuadTree(KeyPtVector &keypts);

    KeyPtVector distributeQuadTree(const KeyPtVector& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N);
protected:
    cv::Ptr<cv::Feature2D>  mFeature;
    Position::FloatVector   mSigmaVector;
};




#endif