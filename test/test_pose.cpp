#include "test.h"
#include "P_Factory.h"
#include "project/newhwproject.h"
#include "P_IOHelper.h"
#include "P_Task.h"
#include "P_Utils.h"





TESTBEGIN()

    CHECKPARAMSIZE(4)
    {
        PROMT_V("Usage",TEST_PARAM1,"ImgPath","Img1","Img2");
        return -1;
    }
    // "/media/tu/Work/Datas/9-200527-02/Output/Pic/"  "0-000006-687-0001459.jpg" "0-000006-781-0001461.jpg"

    std::shared_ptr<Position::IConfig> pcfg = Position::IConfig::CreateDefaultInstance();
    //set logger
    LOG_INITIALIZE(pcfg);

    Position::CameraParam cam;

    cam.K = (Mat_<MATTYPE>(3,3) <<  1197.087918053, 0, 992.219633496,
                                    0, 1197.559963210, 578.406356694,
                                    0, 0, 1);

    // cam.D = (Mat_<MATTYPE>(4,1) << -0.325687861,0.09756899400000001,0.000333549,9.04e-07);
    cam.D = (Mat_<MATTYPE>(4,1) << -0.326838808,0.097945517,-0.000018248,0.000663664,0.000000000);
    //set camera param
    pcfg->setCamera(cam);
    //set imgpath
    SETCFGVALUE(pcfg,ImgPath,string(TEST_PARAM2));
    //
    Position::FrameData *fst = new Position::FrameData(TEST_PARAM3);
    Position::FrameData *sec = new Position::FrameData(TEST_PARAM4);

    fst->_pos._yaw      = 80.626;
    fst->_pos._pitch    = -0.725;
    fst->_pos._roll     = -0.326;
    fst->_pos.pos       = {30.4562540, 114.4438620, 29.391};

    sec->_pos._yaw      = 80.569;
    sec->_pos._pitch    = -0.816;
    sec->_pos._roll     = -0.314;
    sec->_pos.pos       = {30.4562548,114.4438678, 29.392};


    Mat cam2imuR = (Mat_<MATTYPE>(3,3) << 0.999950047,0.000041058,-0.009995098,
                                           0.009995104,-0.008084173,0.999917369,
                                           -0.000039748,-0.999967322,-0.008084179);

    cout << Position::PCoorTrans::RotationMatrixToEulerAngles(cam2imuR) << endl;
    // cam2imuR = Mat::eye(3,3,MATCVTYPE);

    Mat cam2imuT = (Mat_<MATTYPE>(3,1) << -0.054535300, 0.138047000, -0.026901100);

    cam2imuT = Mat::zeros(3,1,MATCVTYPE);
    Mat R,t;

    Position::PUtils::ComputeRtFromPose(fst->_pos,sec->_pos,cam2imuR,cam2imuT,R,t);

    Position::PoseFlowTask task(cam);

    Position::PoseFlowTask::Item item(fst,sec);

    const Position::PoseResult &result = task.run(item);
    PROMT_V("Pose",result._R) ;

    PROMT_S("R Dis");
    cout << Position::PCoorTrans::RotationMatrixToEulerAngles(R) << endl;
    cout << Position::PCoorTrans::RotationMatrixToEulerAngles(result._R) << endl;
     
    PROMT_S("t Dis");
    cout << t << endl;
    cout << result._t << endl;
    
    const Mat &rR = R;
    const Mat &tT = t;
    Position::EpLine line = Position::PUtils::ComputeEpLine(rR,tT,cam,Point2f(1099,620));
    Position::PUtils::DrawEpiLine(line,Point2f(1105,619),sec->_img);


    line = Position::PUtils::ComputeEpLine(rR,tT,cam,Point2f(984,387));
    Position::PUtils::DrawEpiLine(line,Point2f(988,380),sec->_img);


    imshow("test",sec->_img);
    // pt
    waitKey(0);

TESTEND()