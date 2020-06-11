#include "test.h"
#include "P_Factory.h"
#include "project/newhwproject.h"
#include "P_IOHelper.h"
#include "P_Task.h"

TESTBEGIN()

    CHECKPARAMSIZE(4)
    {
        PROMT_V("Usage",TEST_PARAM1,"ImgPath","Img1","Img2");
        return -1;
    }
    
    std::shared_ptr<Position::IConfig> pcfg = Position::IConfig::CreateDefaultInstance();
    //set logger
    LOG_INITIALIZE(pcfg);

    Position::CameraParam cam;

    cam.K = (Mat_<MATTYPE>(3,3) <<  1198.724601382, 0, 956.076294531,
                                    0, 1199.057323106, 614.758442518,
                                    0, 0, 1);

    cam.D = (Mat_<MATTYPE>(4,1) << -0.325687861,0.09756899400000001,0.000333549,9.04e-07);
    //set camera param
    pcfg->setCamera(cam);
    //set imgpath
    SETCFGVALUE(pcfg,ImgPath,string(TEST_PARAM2));
    //
    Position::FrameData *fst = new Position::FrameData(TEST_PARAM3);
    Position::FrameData *sec = new Position::FrameData(TEST_PARAM4);

    Position::PoseFlowTask task(cam);

    Position::PoseFlowTask::Item item(fst,sec);

    PROMT_V("Pose",task.run(item)._R) ;
TESTEND()