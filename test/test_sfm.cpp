#include "test.h"
#include "project/weiyaproject.h"
#include "P_IOHelper.h"
#include "P_Factory.h"
#include "P_MapDisplay.h"
#include "P_Factory.h"

TESTBEGIN()
#if USE_VIEW

    std::shared_ptr<Position::IConfig> pCfg(new WeiyaConfig("../config/config_weiya.yaml"));

    std::shared_ptr<Position::IFrameData>   pData(new WeiyaData(pCfg));

    LOG_INITIALIZE(pCfg);
    SETGLOBALCONFIG(pCfg);
    SETCFGVALUE(pCfg,ViewEnable,1);

    Position::PMapDisplay mapDisplay(pData,pCfg,2);
    mapDisplay.run();
#endif
TESTEND()