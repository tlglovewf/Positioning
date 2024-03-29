/**
 *   P_ORBFeature.h
 *   
 *   add by tu ligen   2020.2.14
 * 
 */
#ifndef __PORBFEATURE_H_H_
#define __PORBFEATURE_H_H_
#include "P_Interface.h"

namespace Position
{

#pragma region ORBFEATURE
class ORBextractor;

//! orb feature detect
class ORBFeature : public IFeature
{
public:
    //! 计算特征点 以及 描述子
    virtual bool detect(const FrameData &frame,FeatureInfo &info);
protected:
    //! 初始化
    virtual void init();

protected:
    std::unique_ptr<ORBextractor> mpExtractor;
};

#pragma region ORBextractor

class ExtractorNode
{
public:
    ExtractorNode() : bNoMore(false) {}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    KeyPtVector vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    enum
    {
        HARRIS_SCORE = 0,
        FAST_SCORE = 1
    };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor() {}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()(cv::InputArray image, cv::InputArray mask,
                    KeyPtVector &keypoints,
                    cv::OutputArray descriptors);

    int inline GetLevels()
    {
        return nlevels;
    }

    float inline GetScaleFactor()
    {
        return scaleFactor;
    }

    inline const FloatVector &GetScaleFactors() const
    {
        return mvScaleFactor;
    }

    inline const FloatVector &GetInverseScaleFactors() const
    {
        return mvInvScaleFactor;
    }

    inline const FloatVector &GetScaleSigmaSquares() const
    {
        return mvLevelSigma2;
    }

    inline const FloatVector &GetInverseScaleSigmaSquares() const
    {
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:
    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<KeyPtVector> &allKeypoints);
    KeyPtVector DistributeOctTree(const KeyPtVector &vToDistributeKeys, const int &minX,
                                  const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    IntVector mnFeaturesPerLevel;

    IntVector umax;

    FloatVector mvScaleFactor;
    FloatVector mvInvScaleFactor;
    FloatVector mvLevelSigma2;
    FloatVector mvInvLevelSigma2;
};
#pragma endregion //ORBextrator

#pragma endregion //ORBFEATURE


DECLAREIFACTORY(IFeature, ORBFeature,Orb)

} // namespace Position

#endif