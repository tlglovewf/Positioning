#include "test.h"
#include "project/newhwproject.h"
#include "P_IOHelper.h"
#include "P_Factory.h"
#include "P_MapDisplay.h"
#include "P_Mask.h"
TESTBEGIN()
#if USE_VIEW
    // if(argv < 2)
    // {
    //     PROMT_S("Params Not Enough!!!");
    //     PROMT_S("Please input cmd prjpath.");
    //     return -1;
    // }

    std::shared_ptr<Position::IConfig> pCfg(new ImgAutoConfig("../config/config_new.yaml"));

    std::shared_ptr<Position::IFrameData>   pData(new NewHwProjectData(pCfg));

    LOG_INITIALIZE(pCfg);
    SETGLOBALCONFIG(pCfg);
    // SETCFGVALUE(pCfg, PrjPath, prjpath + "/Output");
    SETCFGVALUE(pCfg,ViewEnable,1);

#if 1

    int width  = GETCFGVALUE(pCfg,ImgWd,int);
    int height = GETCFGVALUE(pCfg,ImgHg,int);
    //set global mask
    INITGLOBALMASK(Size2i(width, height));

    const int mklen = 250;
    SETGLOBALMASK(Rect2i(0,height - mklen,width,mklen));

    Position::PMapDisplay mapDisplay(pData,pCfg,1);
    mapDisplay.run();

#else
    pData->loadDatas();
    string imgpath = GETCFGVALUE(pCfg,ImgPath,string);
    for(Position::FrameDataPtrVIter it = pData->begin(); it != pData->end(); it++)
    {
        Mat img = imread(imgpath + "/" + (*it)->_name);
        cout <<  (*it)->_name.c_str()  << endl;
        if( !img.empty() && 
            !pCfg->getCamera().D.empty())
        {
            Mat oimg;
            undistort(img,oimg,pCfg->getCamera().K,pCfg->getCamera().D);
            resize(oimg,oimg,Size(oimg.cols >> 1, oimg.rows >> 1));
            imshow("test",oimg);
            // imwrite("/media/tu/Work/Datas/9-200524-00/Output/" + (*it)->_name,oimg);
            waitKey(100);
        }
    }

#endif

    
    
#endif
TESTEND()