
#include "P_Factory.h"
#include "P_Writer.h"
#include "P_Utils.h"
#include "project/imgautoproject.h"
#include "P_MapDisplay.h"


int main(void)
{
    std::shared_ptr<Position::IConfig> pCfg(new Position::ImgAutoConfig("../config/config_auto.yaml"));    
    std::shared_ptr<Position::IData>   pData(new Position::ImgAutoData(pCfg));

#if 0

    std::unique_ptr<PMapDisplay> system(new PMapDisplay(pData,pCfg));

    Position::Time_Interval t;
    t.start();
    system->run();
    t.prompt("total cost : ");
#else

    if(!pData->loadDatas())
    {
        PROMTD_S("Project datas have failed to load!");
        return -1;
    }

    std::shared_ptr<Position::IPositioning> pPosition(Position::PFactory::CreatePositioning(Position::ePMultiImage,pData->getCamera()));

    Position::FrameDataVIter iter = pData->begin();
    Position::FrameDataVIter ed   = pData->end();

    const std::string imgpath = GETCFGVALUE(pCfg,ImgPath, string);
    if(imgpath.empty())
        return -1;  

    Position::ImgAutoPrjList prjlist;
    prjlist.loadPrjList("/media/tlg/work/tlgfiles/WEIYA/9900000120042200/tracker/tracker.txt");
    prjlist.saveMap("/media/tlg/work/tlgfiles/WEIYA/9900000120042200/config/test.txt");

    // for(; iter != ed; ++iter)
    // {
    //     if(iter->_targets.empty())
    //     {
    //         continue;
    //     }
    //     Mat image = imread(imgpath + iter->_name);
    //     for(auto item : iter->_targets)
    //     {
    //         const Rect2f rect = item._box;
    //         cv::rectangle(image,rect,CV_RGB(0,255,0),3);
    //     }
    //     resize(image,image,Size(image.cols >> 2,image.rows >> 2));
    //     imshow("image", image);
    //     waitKey(5);
    // }
#endif

    return 0;
}