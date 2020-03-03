#include "P_Factory.h"
#include "P_BlockMatcher.h"
#include "P_ORBFeature.h"

namespace Position
{
     /*
     * 创建对象
     */
    IBlockMatcher* PFactory::CreateBlockMatcher(eBlockMatcherType type,const Mat &img, const Point2f &pt)
    {
        switch (type) {
            case eNCC:
                return new NCC_BlockMatcher(img,pt);
                //add more
            default:
                return NULL;
        }
    }

    /*
    * 特征点
    */
    IFeature* PFactory::CreateFeature(eFeatureType type, std::shared_ptr<IConfig> pcfg)
    {
        switch(type)
        {
            case eOrbFeature:
                return new ORBFeature(pcfg);
            default:
                return NULL;
        }
    }

    

}