/*
    地图文件可视化
*/

#include "P_Config.h"
#include "P_IOHelper.h"
#include "P_Factory.h"
int main(int argv, char **argc)
{
#if USE_VIEW

    if(argv < 4)
    {
        PROMT_S("Params Not Enough~~");
        PROMT_S("Params Input Like:")
        PROMT_S("mapdisplay ConfigPath FrameFile PointsFile");
        return -1;
    }

    std::shared_ptr<Position::IConfig> pCfg(new Position::PConfig());
    pCfg->load(
        argc[1]
    );

    SETGLOBALCONFIG(pCfg);
    SETCFGVALUE(pCfg,ViewEnable,1);

    DISPLAYMAP(pCfg,argc[2], argc[3])

#endif
    return 0;
}