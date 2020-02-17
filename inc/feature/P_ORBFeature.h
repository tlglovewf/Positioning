/**
 *   P_ORBFeature.h
 *   
 *   add by tu li gen   2020.2.14
 * 
 */
#ifndef __PORBFEATURE_H_H_
#define __PORBFEATURE_H_H_
#include "P_Feature.h"

namespace Position {

    class ORBextractor;

     //orb feature detect
    class ORBFeature : public PFeature
    {
    public:
        ORBFeature(std::shared_ptr<IConfig> pcfg):PFeature(pcfg){}
        //计算特征点 以及 描述子
        virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript);
    protected:
        //初始化
        virtual void init();

    protected:

        std::unique_ptr<ORBextractor> mpExtractor;

    };

#pragma region ORBextractor

    class ExtractorNode
    {
    public:
        ExtractorNode():bNoMore(false){}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    class ORBextractor
    {
    public:

        enum {HARRIS_SCORE=0, FAST_SCORE=1 };

        ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                     int iniThFAST, int minThFAST);

        ~ORBextractor(){}

        // Compute the ORB features and descriptors on an image.
        // ORB are dispersed on the image using an octree.
        // Mask is ignored in the current implementation.
        void operator()( cv::InputArray image, cv::InputArray mask,
                    KeyPtVector & keypoints,
          cv::OutputArray descriptors);

        int inline GetLevels(){
            return nlevels;}

        float inline GetScaleFactor(){
            return scaleFactor;}

        std::vector<float> inline GetScaleFactors(){
            return mvScaleFactor;
        }

        std::vector<float> inline GetInverseScaleFactors(){
            return mvInvScaleFactor;
        }

        std::vector<float> inline GetScaleSigmaSquares(){
            return mvLevelSigma2;
        }

        std::vector<float> inline GetInverseScaleSigmaSquares(){
            return mvInvLevelSigma2;
        }

        std::vector<cv::Mat> mvImagePyramid;

    protected:

        void ComputePyramid(cv::Mat image);
        void ComputeKeyPointsOctTree(std::vector<KeyPtVector>& allKeypoints);    
        KeyPtVector DistributeOctTree(const KeyPtVector& vToDistributeKeys, const int &minX,
                                               const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

        void ComputeKeyPointsOld(std::vector<KeyPtVector>& allKeypoints);
        std::vector<cv::Point> pattern;

        int nfeatures;
        double scaleFactor;
        int nlevels;
        int iniThFAST;
        int minThFAST;

        std::vector<int> mnFeaturesPerLevel;

        std::vector<int> umax;

        std::vector<float> mvScaleFactor;
        std::vector<float> mvInvScaleFactor;    
        std::vector<float> mvLevelSigma2;
        std::vector<float> mvInvLevelSigma2;
    };
#pragma endregiion
} 

#endif