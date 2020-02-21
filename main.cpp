#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching.hpp>

#include <vector>

#include "P_Controller.h"

#include "P_ORBFeature.h"
#include "P_Config.h"
#include "P_FeatureMatcher.h"
#include "P_Frame.h"
#include "P_PoseEstimation.h"
#include "P_Data.h"

using namespace std;
using namespace cv;

int main(void)
{   
    // PositionController pcontroller("../config.yaml",PositionController::eWeiyaType);

    // //初始化信息
    // if(pcontroller.init())
    // {
    // }

    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<Position::WeiyaConfig>(); 
    pCfg->load("../config.yaml");
    

    const string imgpath = GETCFGVALUE( (*pCfg)["ImgPath"] ,string);//
    Mat img1 = imread(imgpath + "/20191107-072927356003-0000000300_L.jpg",IMREAD_UNCHANGED);
    Mat img2 = imread(imgpath + "/20191107-072928328188-0000000301_L.jpg",IMREAD_UNCHANGED);

    Position::FrameData frame1;
    Position::FrameData frame2;
    frame1._img = img1;
    frame2._img = img2;

    std::unique_ptr<Position::IData> pData(new Position::WeiyaData(pCfg));
    pData->loadDatas();

    std::shared_ptr<Position::IFeature> pFeature = std::make_shared<Position::ORBFeature>(pCfg);

    Position::IFrame *preframe = new Position::PFrame(frame1,pFeature);
    Position::IFrame *curframe = new Position::PFrame(frame2,pFeature);

    Position::FrameGrid::initParams(img1.cols,img1.rows);
    Position::FrameGrid::assignFeaturesToGrid(preframe);
    Position::FrameGrid::assignFeaturesToGrid(curframe);
    

    Ptr<Position::IFeatureMatcher> pMatcher = new Position::PFeatureMatcher(0.9);

    Position::MatchVector matches = pMatcher->match(preframe,curframe,GETCFGVALUE((*pCfg)["SearchScale"],int));

    if(matches.empty())
    {
        std::cout << "error ." << std::endl;
    }
    else
    {
        std::cout << "matches size " << matches.size() << std::endl;
        // Mat oimg;
        // cv::drawMatches(frame1._img,preframe->getKeys(),frame2._img,curframe->getKeys(),matches,oimg);

        // imwrite("/Users/TLG/Documents/Result/result.jpg",oimg);

        // cout << "write file successfully." << endl;

        std::unique_ptr<Position::IPoseEstimation> pPoseEst(new Position::CVPoseEstimation());// new Position::ORBPoseEstimation());

        pPoseEst->setCamera(pData->getCamera());

        pPoseEst->setParams(preframe,curframe,matches);
        Mat R,t;
        if(pPoseEst->estimate(R,t))
        {
            cout << "R " << R << endl;
            cout << "t " << t << endl;
        }
        else
        {
            cout << "pose estimate failed." << endl;
        }
      }

    return 0;
}