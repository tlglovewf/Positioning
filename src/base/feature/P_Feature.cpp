#include "P_Feature.h"

namespace Position
{
    PCVORBFeature::PCVORBFeature(const std::shared_ptr<IConfig> &pcfg):PFeature(pcfg)
    {
      init();
    }

    void PCVORBFeature::init()
    {
        int   nfeatures = GETCFGVALUE(mCfg,FeatureCnt,int);
        int   nlevel    = GETCFGVALUE(mCfg,PyramidLevel,int);
        float scale     = GETCFGVALUE(mCfg,ScaleFactor,float);

        mpFeature = ORB::create(nlevel, scale, nlevel ,31, 0, 2, ORB::HARRIS_SCORE,31,20);
    }

    //计算特征点
    bool PCVORBFeature::detect(const FrameData &frame,KeyPtVector &keys, Mat &descript)
    {
        assert(mpFeature);
        if(frame._img.empty())
            return false;
        mpFeature->detectAndCompute(frame._img,Mat(),keys,descript);
        return true;
    }
}
