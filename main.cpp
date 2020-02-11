#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching.hpp>

#include <vector>

#include "P_Config.h"

using namespace std;
using namespace cv;

int main(void)
{   
    Ptr<Position::IConfig> pconfig(new Position::PConfig);
    pconfig->load("../config.yaml");
    cout << GETCFGVALUE((*pconfig)["EdNo"],int) << endl;
    cout << GETCFGVALUE((*pconfig)["OutPath"],string).c_str() << endl;
    // (*reinterpret_cast<int*>(pconfig["EdNo"]->data()));
    return 0;
}