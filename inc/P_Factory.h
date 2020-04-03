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
#define DEFINEENUM(ENUM)  enum e##ENUM##Type
#define DEFINEFUNC(FUNC)  static I##FUNC* Create##FUNC(e##FUNC##Type type); 
    /*
     * 块匹配算法
     */
    DEFINEENUM(BlockMatcher)
    {
        eBMNCC,
        eBMSAD,
        eBMSSD
    }; 

    /*
     * 特征提取类型
     */
    DEFINEENUM(Feature)
    {
        eFeatureOrb,    //orbslam 
        eFeatureCVOrb   //cv orb
    };

    DEFINEENUM(FeatureMatcher)
    {
        eFMDefault
    };

    /*
     * 位姿估算
     */
    DEFINEENUM(PoseEstimation)
    {
        ePoseEstOrb,
        ePoseEstCV
    };

    /*
     * 优化类型
     */
    DEFINEENUM(Optimizer)
    {
        eOpG2o
    };

    /*
     * 定位类型
     */
    DEFINEENUM(Positioning)
    {
        ePSingleImage,
        ePMultiImage,
        ePDepthImage
    };

    /*
     * 可视化类型
     */
    DEFINEENUM(Viewer)
    {
        eVPangolin
    };

    /*
     * 跟踪类型
     */
    DEFINEENUM(TrajProcesser)
    {
        eUniformSpeed,   //匀速运动模型
        eMultiVision     //多视图场景
    };

    /*
     * 检查类型
     */
    DEFINEENUM(Checker)
    {
        eNormalChecker
    };

    class IOptimizer;
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
        static IFeature* CreateFeature(eFeatureType type,const std::shared_ptr<IConfig> &pcfg);

        /*
         * 创建特征匹配对象
         */
        static IFeatureMatcher* CreateFeatureMatcher(eFeatureMatcherType type, float ratio, bool bcheckori = true);

        /*
         * 创建定位对象
         */
        static IPositioning* CreatePositioning(ePositioningType type, const CameraParam  &pcfg);
        
        /*
         * 创建可视化
         */
        static IViewer* CreateViewer(eViewerType type,const std::shared_ptr<IConfig> &pcfg);

        /*
         * 创建轨迹处理对象
         */
        static ITrajProcesser* CreateTrajProcesser(eTrajProcesserType type, 
                                                   const std::shared_ptr<IConfig> &pcfg,
                                                   const std::shared_ptr<IData> &pdata);

        /*
         * 位姿推算
         */
        DEFINEFUNC(PoseEstimation)

        /*
         * 优化
         */
        DEFINEFUNC(Optimizer)

        /*
         * 检查
         */
        DEFINEFUNC(Checker)

    };

    //持久化 轨迹枚举
    enum eMapTraceSerType
    {
        eDefaultTraceSer
    };

    //持久化 地图点
    enum eMapPointSerType
    {
        eDefaultPtSer
    };

    //地图持久化管理类
    class MapSerManager
    {
    public:
        MapSerManager();
        //单例
        static MapSerManager* Instance()
        {
            static MapSerManager mgr;
            return &mgr;
        }

        //设置地图
        void setMap(const std::shared_ptr<Position::IMap> &pmap)
        {
            mpTracSer->setMap(pmap);
            mpPtSer->setMap(pmap);
        }

        //设置持久化类型
        void SetSerType(eMapTraceSerType trtype, eMapPointSerType epttype);

        //获取轨迹持久化指针
        ISerialization* tracSerPtr()
        {
            return  mpTracSer;
        }
        //获取地图点持久化指针
        ISerialization* mpPtSerPtr()
        {
            return mpPtSer;
        }

        //显示地图
        void displayMap(const std::shared_ptr<IConfig> &pCfg, const std::string &trac,const std::string &mpts);

    protected:
        ISerialization *mpTracSer;
        ISerialization *mpPtSer;
    };
}

#endif