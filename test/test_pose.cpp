#include "test.h"
#include "P_Factory.h"
#include "project/newhwproject.h"
#include "P_Frame.h"
//任务
class ITask
{
public:
    //! 执行
    virtual void run() = 0;
};

class PoseTask
{
public: 
    struct Item
    {
        Position::FrameData *queryItem;
        Position::FrameData *trainItem;
    };
    
};

TESTBEGIN()
    std::shared_ptr<Position::IConfig>          pcfg(new ImgAutoConfig("../config/config_new.yaml"));
    SETGLOBALCONFIG(pcfg);

    std::shared_ptr<Position::IFeature>         pfeature(GETFEATURE(Uniform));
    std::shared_ptr<Position::IFeatureMatcher>  pMatcher(GETFEATUREMATCHER(Knn));
    std::shared_ptr<Position::IPoseSolver>      pEst(GETPOSESOLVER(CVPoseSolver));
    std::shared_ptr<Position::IFrameData>       pdata(new NewHwProjectData(pcfg));

    pdata->loadDatas();
    int bgno = GETCFGVALUE(pcfg,StNo,int);
    
    Position::FrameDataPtrVIter fst = pdata->begin() + bgno;
    Position::FrameDataPtrVIter sec = fst + 2;

    Position::FeatureInfo info1((*fst)->_name);
    Position::FeatureInfo info2((*sec)->_name);

    string imgpath = GETCFGVALUE(pcfg,ImgPath,string) + "/";
    (*fst)->_img = imread(imgpath + (*fst)->_name);
    (*sec)->_img = imread(imgpath + (*sec)->_name);


    pfeature->detect(**fst,info1);
    pfeature->detect(**sec,info2);
    Position::MatchVector matches = std::move(pMatcher->match(info1,info2,3));

    cout << matches.size() << endl;

    Position::InputPair input(info1._keys,info2._keys,matches);
    pEst->setCamera(pcfg->getCamera());
    Position::PoseResult result = pEst->estimate(input);

    cout << result._R << endl;
    cout << result._t << endl;

TESTEND()