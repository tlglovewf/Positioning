/**
 *   P_Factory.h
 *   
 *   add by tu li gen   2020.2.14
 * 
 */
#ifndef __PFACTORY_H_H_
#define __PFACTORY_H_H_
#include "P_Interface.h"
#include "P_IOHelper.h"
namespace Position
{   
    template<typename F>
    //! 基础工厂方法
    class BaseFactoryMethod
    {
    public: 
        //！ 创建相关
        std::shared_ptr<F>  create(const std::string &name)
        {
            if(mItems.find(name) == mItems.end())
            {
                LOG_CRIT_F("%s Not Found Please Check it.",name.c_str());
                exit(-1);
            }
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
     * 特征点工厂 "Orb" "Sift" "SiftEx"
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
#define CREATEFACTORYINSTANCE(F,N)  Position::I##F##Factory::Instance()->create(N)
#define GETFEATURE(N)               CREATEFACTORYINSTANCE(Feature,N)
#define GETFEATUREMATCHER(N)        CREATEFACTORYINSTANCE(FeatureMatcher,N)
#define GETPOSESOLVER(N)            CREATEFACTORYINSTANCE(PoseSolver,N)
#define GETTRJPROCESSER(N)          CREATEFACTORYINSTANCE(TrajProcesser,N)

#define GETVIEWER()                 CREATEFACTORYINSTANCE(Viewer,"Pangolin")
#define GETOPTIMIZER()              CREATEFACTORYINSTANCE(Optimizer,"G2o")
}

#endif