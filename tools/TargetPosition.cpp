
#include "P_Factory.h"
#include "P_Writer.h"
#include "P_Utils.h"
#include "project/imgautoproject.h"
#include "PosBatchHandler.h"
#include "P_Checker.h"
int main(int argc, char **argv)
{
    if(2 != argc)
    {
        PROMTD_S("Paramer's size error!");
        PROMTD_S("Please input absolutely path");
        PROMTD_V("Format","TargetPosition \"PROJROOTPATH\" ");
        return -1;
    }

    if(!PATHCHECK(argv[1]))
    {
        PROMTD_V(argv[1], "Dirtroy not found! Please Check it.");
        return -1;
    }
    PROMTD_V("Project Path",argv[1]);
    string prjpath = string(argv[1]) + "/";
    string cfgpath = prjpath + "config/config.yaml";

    if(!PATHCHECK(cfgpath))
    {
        PROMTD_V(cfgpath.c_str(), "Config not found! Please Check it.");
        return -1;
    }

    
    PROMTD_V("Config Path",cfgpath.c_str());

    
    std::shared_ptr<Position::IConfig> pCfg(new Position::ImgAutoConfig(cfgpath)); 
    SETCFGVALUE(pCfg,PrjPath,prjpath);
    SETCFGVALUE(pCfg,CamMatrixPath,string(prjpath + "config/extrinsics.xml"));
    std::shared_ptr<Position::IData>   pData(new Position::ImgAutoData(pCfg));

    if(!pData->loadDatas())
    {
        PROMTD_S("Failed to load the Project datas!");
        return -1;
    }


    Position::FrameDataVIter iter = pData->begin();
    Position::FrameDataVIter ed   = pData->end();

  

#if 0  //only for test
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
#endif

    PosBatchHandler poshandler(pCfg,pData);
    const std::string trackerpath = prjpath + "tracker/";
    if(poshandler.loadTrackerInfos(trackerpath + "tracker.txt"))
    {
        //先计算位姿
        poshandler.poseEstimate();
        //目标定位 
        poshandler.targetPositioning();
        //结果输出
        poshandler.saveResult(trackerpath + "tracker_rst.txt");
    }
    
    return 0;
}