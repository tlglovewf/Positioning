
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
    //std::shared_ptr<Position::IPositioning> pPosition(Position::PFactory::CreatePositioning(Position::ePMultiImage,pData->getCamera()));

    Position::FrameDataVIter iter = pData->begin();
    Position::FrameDataVIter ed   = pData->end();

    const std::string imgpath = GETCFGVALUE(pCfg,ImgPath, string);
    if(imgpath.empty())
        return -1;
    for(; iter != ed; ++iter)
    {
        if(iter->_targets.empty())
        {
            continue;
        }
        Mat image = imread(imgpath + iter->_name);
        for(auto item : iter->_targets)
        {
            const Rect2f rect = item._box;
            cv::rectangle(image,rect,CV_RGB(0,255,0),3);
        }
        resize(image,image,Size(image.cols >> 2,image.rows >> 2));
        imshow("image", image);
        waitKey(5);
    }


    return 0;
}