#include "project/weiyaproject.h"
#include "P_IOHelper.h"
#include "P_Factory.h"
#include "P_MapDisplay.h"
int main(int argv, char **argc)
{
// #if USE_VIEW

    std::shared_ptr<Position::IConfig> pCfg(new WeiyaConfig("../config/config_weiya.yaml"));

    std::shared_ptr<Position::IData>   pData(new WeiyaData(pCfg));


    LOG_INITIALIZE(pCfg);
    SETGLOBALCONFIG(pCfg);
    SETCFGVALUE(pCfg,ViewEnable,1);

    PMapDisplay mapDisplay(pData,pCfg,1);
    mapDisplay.run();
// #endif
    return 0;
}