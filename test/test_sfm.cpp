#include "test.h"
#include "project/weiyaproject.h"
#include "P_IOHelper.h"
#include "P_Factory.h"
#include "P_MapDisplay.h"
#include "P_Factory.h"
#include "P_SiftFeature.h"
#include "P_Utils.h"
#include "Thirdparty/filesystem/include/ghc/filesystem.hpp"

TESTBEGIN()
// #if USE_VIEW

//     std::shared_ptr<Position::IConfig> pCfg(new WeiyaConfig("../config/config_weiya.yaml"));

//     std::shared_ptr<Position::IFrameData>   pData(new WeiyaData(pCfg));

//     LOG_INITIALIZE(pCfg);

//     SETCFGVALUE(pCfg,ViewEnable,1);

//     Position::PMapDisplay mapDisplay(pData,pCfg,2);
//     mapDisplay.run();
// #endif

    ghc::filesystem::path  picpath("/media/tu/Work/GitHub/TwoFrameSO/data/inputim");
    CREATEGLOBALCONFIG();
    Position::Time_Interval time;
    time.start();
    
    if( ghc::filesystem::exists(picpath)     &&
        ghc::filesystem::is_directory(picpath) )
    {
        for( auto item : ghc::filesystem::directory_iterator(picpath))
        {
            if(item.is_regular_file())
            {
                std::cout << item << std::endl;
                
                cv::Mat img = cv::imread(item.path().c_str(),CV_LOAD_IMAGE_UNCHANGED);
                Position::PUtils::ShowImage("test",img);
                Position::SiftFeature sift;
                Position::FrameData framedata;
                framedata._img = img;
                Position::FeatureInfo info(item.path().filename());
                Position::Time_Interval timer;
                timer.start();
                // sift.detect(framedata,info);
                
                // sift.compute(img,info._keys,info._des);
                sift.detectAndCompute(framedata._img,noArray(),info._keys,info._des);
                timer.prompt("cost");
            }
        }
    }
    time.prompt("total cost ");
    
    

TESTEND()