#include "P_MapDisplay.h"
#include "P_Factory.h"
#include "P_IOHelper.h"

#include "P_MultiVisionTrajProcesser.h"

#include "project/hdproject.h"
#include "project/weiyaproject.h"

#include "P_Map.h"
#include "P_Factory.h"
#include "P_MapSerializor.h"
#include "P_Utils.h"
#include "feature/P_SiftFeature.h"
#include "P_FrameViewer.h"

#include "P_PangolinViewer.h"

#include "P_SemanticGraph.h"

#include "P_GpsFusion.h"

#include <thread>

#include "Thirdparty/GeographicLib/include/LocalCartesian.hpp"
#include "P_CoorTrans.h"


using namespace std;
using namespace cv;

#define WEIYA           0  //是否为weiya数据
#define USECONTROLLER   1  //是否启用定位框架

int main(void)
{  
    cv::Ptr<Position::SiftFeature> sift = Position::SiftFeature::create(2000);

    cv::Mat img = imread("/media/tu/Work/GitHub/TwoFrameSO/data/inputim/0-006437-467-0007818.jpg",CV_LOAD_IMAGE_UNCHANGED);

    Position::FrameData fmdata;
    fmdata._img = img;
    Position::FeatureInfo fminfo("1");
    Position::Time_Interval timer;
    timer.start();
    sift->detect(fmdata,fminfo);
    sift->compute(fmdata._img,fminfo._keys,fminfo._des);
    // sift->detectAndCompute(fmdata._img,noArray(),fminfo._keys,fminfo._des);
    timer.prompt("cost",true);
    cv::Mat mt = Position::PUtils::DrawKeyPoints(img,fminfo._keys);

    Position::PUtils::ShowImage("test",mt);

    cv::Mat vt;

    double sigma = 0.01;

    cv::GaussianBlur(img,vt,cv::Size(),sigma,sigma);
    timer.prompt("gauss",true);
    Position::PUtils::ShowImage("gauss",vt);
    // cv::imwrite("/media/tu/Work/Datas/newdata/result.jpg",mt);

    cv::waitKey(0);

    return 0;
}