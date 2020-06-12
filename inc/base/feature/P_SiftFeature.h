/**
 *   P_SiftFeature.h
 *   
 *   add by tu li gen   2020.6.3
 * 
 */

#ifndef __SIFTFEATURE_H_H_
#define __SIFTFEATURE_H_H_
#include "P_Interface.h"

namespace Position
{
    //! Sift
    class SiftFeature : public Feature2D,public IFeature
    {
    public:
        //! 构造函数
        SiftFeature();

        //! 计算特征点
        virtual bool detect(const FrameData &frame,FeatureInfo &info);

        public:
        /**
        @param nfeatures The number of best features to retain. The features are ranked by their scores
        (measured in SIFT algorithm as the local contrast)

        @param nOctaveLayers The number of layers in each octave. 3 is the value used in D. Lowe paper. The
        number of octaves is computed automatically from the image resolution.

        @param contrastThreshold The contrast threshold used to filter out weak features in semi-uniform
        (low-contrast) regions. The larger the threshold, the less features are produced by the detector.

        @param edgeThreshold The threshold used to filter out edge-like features. Note that the its meaning
        is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are
        filtered out (more features are retained).

        @param sigma The sigma of the Gaussian applied to the input image at the octave \#0. If your image
        is captured with a weak camera with soft lenses, you might want to reduce the number.
        */
        static Ptr<SiftFeature> create( int nfeatures = 0, int nOctaveLayers = 3,
                                double contrastThreshold = 0.04, double edgeThreshold = 10,
                                double sigma = 1.6);

        explicit SiftFeature(int nfeatures, int nOctaveLayers = 3,
                        double contrastThreshold = 0.04, double edgeThreshold = 10,
                        double sigma = 1.6);

        //! returns the descriptor size in floats (128)
        int descriptorSize() const;

        //! returns the descriptor type
        int descriptorType() const;

        //! returns the default norm type
        int defaultNorm() const;

        //! finds the keypoints and computes descriptors for them using SIFT algorithm.
        //! Optionally it can compute descriptors for the user-provided keypoints
        void detectAndCompute(InputArray img, InputArray mask,
                            std::vector<KeyPoint> &keypoints,
                            OutputArray descriptors,
                            bool useProvidedKeypoints = false);

        void buildGaussianPyramid(const Mat &base, std::vector<Mat> &pyr, int nOctaves) const;
        void buildDoGPyramid(const std::vector<Mat> &pyr, std::vector<Mat> &dogpyr) const;
        void findScaleSpaceExtrema(const std::vector<Mat> &gauss_pyr, const std::vector<Mat> &dog_pyr,
                                std::vector<KeyPoint> &keypoints) const;

    protected:
        CV_PROP_RW int nfeatures;
        CV_PROP_RW int nOctaveLayers;
        CV_PROP_RW double contrastThreshold;
        CV_PROP_RW double edgeThreshold;
        CV_PROP_RW double sigma;
    };
    DECLAREIFACTORY(IFeature, SiftFeature,Sift)
}

#endif