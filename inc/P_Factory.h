/**
 *   P_Factory.h
 *   
 *   add by tu li gen   2020.2.14
 * 
 */
#ifndef __PFACTORY_H_H_
#define __PFACTORY_H_H_
#include "P_Interface.h"

namespace Position
{   
    /*
     * 块匹配算法
     */
    enum eBlockMatcherType
    {
        eNCC,
        eSAD,
        eSSD
    }; 

    /*
     * 特征提取类型
     */
    enum eFeatureType
    {
        eOrbFeature
    };


#define DEFINEFUNC(FUNC)  static I##FUNC* Create##FUNC(e##FUNC##Type type); 

    //工厂对象
    class PFactory
    {
     public:

        /*
         * 创建对象
         */
        static IBlockMatcher* CreateBlockMatcher(eBlockMatcherType type,const Mat &img, const Point2f &pt);

        /*
         * 特征点
         */
        static IFeature* CreateFeature(eFeatureType type, std::shared_ptr<IConfig> pcfg);
        
    };
}

#endif