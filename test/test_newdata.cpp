#include "project/newhwproject.h"
#include "P_IOHelper.h"
#include "P_Factory.h"
#include "P_MapDisplay.h"
int main(int argv, char **argc)
{
// #if USE_VIEW
    if(argv < 2)
    {
        PROMT_S("Params Not Enough!!!");
        PROMT_S("Please input cmd prjpath.");
        return -1;
    }

    string prjpath(argc[1]);

    std::shared_ptr<Position::IConfig> pCfg(new Position::ImgAutoConfig("../config/config_new.yaml"));

    std::shared_ptr<Position::IData>   pData(new Position::NewHwProjectData(pCfg));

    LOG_INITIALIZE(pCfg);
    SETGLOBALCONFIG(pCfg);
    SETCFGVALUE(pCfg, PrjPath, prjpath + "/Output");
    SETCFGVALUE(pCfg,ViewEnable,1);
#if 1
    PMapDisplay mapDisplay(pData,pCfg,0);
    mapDisplay.run();
#else
    pData->loadDatas();
    string imgpath = GETCFGVALUE(pCfg,ImgPath,string);
    for(Position::FrameDataPtrVIter it = pData->begin(); it != pData->end(); it++)
    {
        Mat img = imread(imgpath + "/" + (*it)->_name);
        cout << imgpath.c_str() << endl;
        if( !img.empty() && 
            !pData->getCamera().D.empty())
        {
            Mat oimg;
            undistort(img,oimg,pData->getCamera().K,pData->getCamera().D);
            resize(oimg,oimg,Size(oimg.cols >> 1, oimg.rows >> 1));
            imshow("test",oimg);
            // imwrite("/media/tu/Work/Datas/9-200524-00/Output/" + (*it)->_name,oimg);
            waitKey(100);
        }
    }

#endif

    
    
// #endif
    return 0;
}