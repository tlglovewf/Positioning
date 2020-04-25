
#include "P_Controller.h"
#include "P_Factory.h"
#include "P_Writer.h"
#include "project/imgautoproject.h"

int main(void)
{
    std::shared_ptr<Position::IConfig> pCfg(new Position::ImgAutoConfig("../config/config_auto.yaml"));    
    std::shared_ptr<Position::IData>   pData(new Position::ImgAutoData(pCfg));
    if(!pData->loadDatas())
    {
        PROMTD_S("Project datas have failed to load!");
        return -1;
    }
    std::shared_ptr<Position::IPositioning> pPosition(Position::PFactory::CreatePositioning(Position::ePMultiImage,pData->getCamera()));

    return 0;
}