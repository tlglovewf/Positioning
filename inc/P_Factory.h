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
     *  持久化 轨迹枚举
     */
    enum eMapTraceSerType
    {
        eDefaultTraceSer
    };

    /*
    *  持久化 地图点
    */
    enum eMapPointSerType
    {
        eDefaultPtSer
    };

     /*
     *  持久化 地图管理类
     */
    class MapSerManager
    {
    public:
        MapSerManager();
        //! 单例
        static MapSerManager* Instance()
        {
            static MapSerManager mgr;
            return &mgr;
        }

        //! 设置地图
        void setMap(const std::shared_ptr<Position::IMap> &pmap)
        {
            mpTracSer->setMap(pmap);
            mpPtSer->setMap(pmap);
        }

        //! 设置持久化类型
        void SetSerType(eMapTraceSerType trtype, eMapPointSerType epttype);

        //! 获取轨迹持久化指针
        ISerialization* tracSerPtr()
        {
            return  mpTracSer;
        }
        //! 获取地图点持久化指针
        ISerialization* mpPtSerPtr()
        {
            return mpPtSer;
        }

        //! 显示地图
        void displayMap(const std::shared_ptr<IConfig> &pCfg, const std::string &trac,const std::string &mpts);

        //! 融合地图  secMap -> baseMap
        void combineMap(std::shared_ptr<IMap> &baseMap, const std::shared_ptr<IMap> &secMap);
    protected:
        ISerialization *mpTracSer;
        ISerialization *mpPtSer;
    };

    template<typename F>
    //! 基础工厂方法
    class BaseFactoryMethod
    {
    public: 
        //！ 创建相关
        std::shared_ptr<F>  create(const std::string &name)
        {
            assert(mItems.find(name) != mItems.end());
            return mItems[name]->create();
        }

    protected:
        NameMap(IBaseFactory<F>)  mItems;
    };
// 工厂定义
#define FACTORYDECLARE(F) \
            class F##Factory : public BaseFactoryMethod<F>\
            {\
            public:\
                F##Factory();\
                static F##Factory* Instance();\
            };

    /* 
     * 特征点工厂 "Orb" "Sift" "Uniform"
     */
    FACTORYDECLARE(IFeature)
    /*
     * 特征匹配工厂 "HanMing" "Knn"
     */
    FACTORYDECLARE(IFeatureMatcher)
    /*
     * 位姿估计类 "CVPoseSolver" "ORBPoseSolver"
     */
    FACTORYDECLARE(IPoseSolver)
    /*
     * 轨迹处理类 "MViewsTraj" "UniformTraj" "SfmTraj"
     */
    FACTORYDECLARE(ITrajProcesser )

    /*
     * 优化接口类 "G2o"
     */
    FACTORYDECLARE(IOptimizer)

#if USE_VIEW
    /*
     * 声明可视化 "Pangolin"
     */
    FACTORYDECLARE(IViewer)
#endif
  
#undef FACTORYDECLARE

/*
 * 获取工厂实例（F工厂类型 N名称)
 */
#define CREATEFACTORYINSTANCE(F,N)  Position::I##F##Factory::Instance()->create(_TOSTRING(N))
#define GETFEATURE(N)               CREATEFACTORYINSTANCE(Feature,N)
#define GETFEATUREMATCHER(N)        CREATEFACTORYINSTANCE(FeatureMatcher,N)
#define GETPOSESOLVER(N)            CREATEFACTORYINSTANCE(PoseSolver,N)
#define GETTRJPROCESSER(N)          CREATEFACTORYINSTANCE(TrajProcesser,N)

#define GETVIEWER()                 CREATEFACTORYINSTANCE(Viewer,Pangolin)
#define GETOPTIMIZER()              CREATEFACTORYINSTANCE(Optimizer,G2o)

#define   SETMAPSERIALIZATIONMAP(MAP)  Position::MapSerManager::Instance()->setMap(MAP);
#define   SAVEMAPFRAME(path)           Position::MapSerManager::Instance()->tracSerPtr()->saveMap(path);
#define   SAVEMAPPOINTS(path)          Position::MapSerManager::Instance()->mpPtSerPtr()->saveMap(path);
#define   LOADMAPFRAME(path)           Position::MapSerManager::Instance()->tracSerPtr()->loadMap(path);
#define   LOADMAPPOINTS(path)          Position::MapSerManager::Instance()->mpPtSerPtr()->loadMap(path);
#define   DISPLAYMAP(CFG,FMS,MPTS)     Position::MapSerManager::Instance()->displayMap(CFG,FMS,MPTS);
}

#endif