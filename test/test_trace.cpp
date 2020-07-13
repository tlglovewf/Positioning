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
    Mat m = imread("/media/tu/Work/Datas/9-200524-00/Output/Pic/0-059362-531-0000001.jpg",CV_LOAD_IMAGE_UNCHANGED);

    std::vector<uchar> imgde;

    imencode(".png",m,imgde);

    std::string outfile(imgde.begin(),imgde.end());

    ofstream file("/media/tu/Work/Datas/img.txt");
    assert(file.is_open());
    file << outfile << endl;
    file.close();



    //Position::PUtils::ShowImage("test",m);


    PROMT_S("Run successfully.");

    // waitKey(0);
    return 0;
}