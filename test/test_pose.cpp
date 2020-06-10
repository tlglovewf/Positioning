#include "test.h"
#include "P_Factory.h"
#include "project/newhwproject.h"
#include "P_IOHelper.h"
#include "P_Task.h"

TESTBEGIN()

    std::shared_ptr<Position::IConfig>          pcfg(new ImgAutoConfig("../config/config_new.yaml"));
    LOG_INITIALIZE(pcfg);
    SETGLOBALCONFIG(pcfg);
    std::shared_ptr<Position::IFrameData>       pdata   (new NewHwProjectData(pcfg));
    pdata->loadDatas();
    int bgno = GETCFGVALUE(pcfg,StNo,int);
    Position::FrameDataPtrVIter fst = pdata->begin() + bgno;
    Position::FrameDataPtrVIter sec = fst + 2;
    
    Position::PoseFlowTask task(pcfg->getCamera());
    Position::PoseFlowTask::Item item(*fst,*sec);

    PROMT_V("Pose",task.run(item)._R) ;
TESTEND()