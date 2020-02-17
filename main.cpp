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

    std::shared_ptr<Position::IFeature> pFeature = std::make_shared<Position::IFeature>(pCfg);
    // Position::KeyPtVector keys1,keys2;
    // Mat des1, des2;
   
    // pFeature->detect(frame1,keys1,des1);
    // pFeature->detect(frame2,keys2,des2);

    // Mat oimg1,oimg2;
    // cv::drawKeypoints(frame1._img,keys1,oimg1);
    // cv::drawKeypoints(frame2._img,keys2,oimg2);

    Position::IFrame *preframe = new Position::PFrame(frame1,pFeature);
    Position::IFrame *curframe = new Position::PFrame(frame2,pFeature);

    Ptr<Position::IFeatureMatcher> pMatcher = new Position::PFeatureMatcher();

    Position::MatchVector matches = pMatcher->match(preframe,curframe,100);

    if(matches.empty())
    {
        std::cout << "error ." << std::endl;
    }
    else
    {
        std::cout << "matches size " << matches.size() << std::endl;
    }

    Mat oimg;
    cv::drawMatches(frame1._img,preframe->getKeys(),frame2._img,curframe->getKeys(),matches,oimg);

    imwrite("",oimg);

    cout << "write file successfully." << endl;

    return 0;
}