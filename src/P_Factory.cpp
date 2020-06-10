#include "P_Factory.h"

#include "P_BlockMatcher.h"
#include "P_ORBFeature.h"
#include "P_SiftFeature.h"
#include "P_FeatureExtend.h"

#include "P_FeatureMatcher.h"

#include "P_PoseSolver.h"
#include "P_Optimizer.h"
#include "P_Positioner.h"

#include "P_Checker.h"

#include "P_PangolinViewer.h"
#include "P_UniformVTrajProcesser.h"
#include "P_MultiVisionTrajProcesser.h"
#include "P_SfmVisonTrajProcesser.h"

#include "P_IOHelper.h"


namespace Position
{
#define FACTORYINSTANCEDECLARE(F)  F##Factory* F##Factory::Instance(){\
        static F##Factory item;\
        return &item;}


    FACTORYINSTANCEDECLARE(IFeature)
    FACTORYINSTANCEDECLARE(IFeatureMatcher)
    FACTORYINSTANCEDECLARE(IPoseSolver)
    FACTORYINSTANCEDECLARE(ITrajProcesser)
    FACTORYINSTANCEDECLARE(IOptimizer)

#if USE_VIEW
    FACTORYINSTANCEDECLARE(IViewer)
#endif


#define INSERT_FACTORY_ITEM(F,T) { std::shared_ptr< Position::IBaseFactory<Position::T> > p(new Position::F##Factory);\
                                mItems.insert(make_pair(p->name(),p));}

    IFeatureFactory::IFeatureFactory()
    {
        INSERT_FACTORY_ITEM(ORBFeature,IFeature)
        INSERT_FACTORY_ITEM(SiftFeature,IFeature)
        INSERT_FACTORY_ITEM(SiftFeatureExtend,IFeature)
    }

    IFeatureMatcherFactory::IFeatureMatcherFactory()
    {
        INSERT_FACTORY_ITEM(HanMingMatcher,IFeatureMatcher)
        INSERT_FACTORY_ITEM(KnnMatcher    ,IFeatureMatcher)
    }

    IPoseSolverFactory::IPoseSolverFactory()
    {
        INSERT_FACTORY_ITEM(CVPoseSolver,IPoseSolver);
        INSERT_FACTORY_ITEM(ORBPoseSolver,IPoseSolver);
    }

    ITrajProcesserFactory::ITrajProcesserFactory()
    {
        INSERT_FACTORY_ITEM(PUniformVTrajProcesser   ,ITrajProcesser)
        INSERT_FACTORY_ITEM(PMultiVisionTrajProcesser,ITrajProcesser)
        INSERT_FACTORY_ITEM(PSfmVisonTrajProcesser   ,ITrajProcesser)
    }

    IOptimizerFactory::IOptimizerFactory()
    {
        INSERT_FACTORY_ITEM(G2oOptimizer,IOptimizer)
    }


#if USE_VIEW
#define INSERT_VIEWER_ITEM(F) { std::shared_ptr<VIEWERFACTORY> p(new F##Factory()); \
        mItems.insert(make_pair(p->name(),p));}
    IViewerFactory::IViewerFactory()
    {
        INSERT_VIEWER_ITEM(Position::PangolinViewer);
    }
#endif
}