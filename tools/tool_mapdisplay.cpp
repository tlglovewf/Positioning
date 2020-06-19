/*
    地图文件可视化
*/

#include "P_Config.h"
#include "P_IOHelper.h"
#include "P_Factory.h"
#include "P_MapSerializor.h"
int main(int argv, char **argc)
{
#if USE_VIEW

    if(argv < 2)
    {
        PROMT_S("Params Not Enough~~");
        PROMT_S("Params Input Like:")
        PROMT_S("1.Display Map Trace And Points...")
        PROMT_S("mapdisplay FrameFile PointsFile");
        PROMT_S("2.Display PrjList Trace...");
        PROMT_S("mapdisplay PrjPoseResult");
        return -1;
    }

    // std::shared_ptr<Position::IConfig> pCfg(new Position::PConfig());
    // pCfg->load(
    //     argc[1]
    // );

    CREATEGLOBALCONFIG();

    // SETGLOBALCONFIG(pCfg);
    SETCFGVALUE(GETGLOBALCONFIG(),ViewEnable,1);

    if(argv == 3)
    {//3个参数 以加载地图的形式
        // DISPLAYMAP(pCfg,argc[2], argc[3])
        DISPLAYMAP(GETGLOBALCONFIG(),argc[1], argc[2])
    }
    else
    {//2个参数 以加载项目列表的形式
        std::shared_ptr<Position::IMap> pMap = Position::PrjWRHelper::Instance()->loadPrjResult(argc[1]);
        if(pMap)
        {
            
            std::shared_ptr<Position::IViewer> pViewer(GETVIEWER());
            pViewer->setMap(pMap);
            pViewer->renderLoop();
        }
        else
        {
            LOG_ERROR("Load PrjList File Failed!!!");
        }
    }
    

#endif
    return 0;
}