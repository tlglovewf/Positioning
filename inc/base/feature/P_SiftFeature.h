/**
 *   P_SiftFeature.h
 *   
 *   add by tu li gen   2020.6.3
 * 
 */

#ifndef __PSIFT_H_H_
#define __PSIFT_H_H_
#include "P_Feature.h"

namespace Position
{
    class PSIFT : public Feature2D, public PFeature
    {
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
        static Ptr<PSIFT> create( int nfeatures = 0, int nOctaveLayers = 3,
                                double contrastThreshold = 0.04, double edgeThreshold = 10,
                                double sigma = 1.6);

         //计算特征点
        virtual bool detect(const FrameData &frame,KeyPtVector &keys, Mat &descript);

        //获取名称
        virtual std::string name()const 
        {
            return _TOSTRING(SIFT);
        }
    };
}

#endif