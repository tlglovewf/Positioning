#include "test.h"
#include "P_Interface.h"
#include "P_IOHelper.h"
#include "P_Checker.h"
#include "P_CoorTrans.h"
#include "P_Utils.h"
#include "project/newhwproject.h"
#include "Thirdparty/sqlite3/sqlite3.h"
#include "P_SemanticGraph.h"
#include "P_Mask.h"
#include "P_ORBFeature.h"
template <typename T>
inline void display(T items)
{
    for (auto i : items)
    {
        cout << i << " ";
    }
    cout << endl;
}

#define _TOSTR(a) #a
#define NMB_STR(a) _TOSTR(a)
#define FILE_LOG() "{" __FILE__ "}"
#define LINE_LOG() "{" NMB_STR(__LINE__) "}"
#define PRINTMESSAGE(X) FILE_LOG() \
LINE_LOG() ":" X

#define FM_T_V "%d" \
               "%s"
#define FM_T_S "%s"

#include <regex>

typedef unsigned int U4;
typedef int L4;


Mat GenerateImageMask(const Mat &img, const Vec3b &clr)
{
    Mat oimg(img.rows,img.cols,CV_8U);
    for(size_t i = 0; i < img.cols; ++i)
    {
        for(size_t j = 0; j < img.rows; ++j)
        {
            if(img.at<Vec3b>(j,i) == clr)
            {
                oimg.at<uchar>(j,i) = 255;
            }
            else
            {
                oimg.at<uchar>(j,i) = 0;
            }
        }
    }
    return oimg;
}


TESTBEGIN()

   Mat img = imread("/media/tu/Work/GitHub/TwoFrameSO/data/segim/0-006437-608-0007821.png",CV_LOAD_IMAGE_UNCHANGED);
   
//    Mat grayimg = GenerateImageMask(img,Vec3b(255,255,255));
   Mat grayimg = GenerateImageMask(img,Vec3b(0,220,220));


  

   std::vector< std::vector< Point> > contours;

   findContours(grayimg,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

   for(size_t i = 0; i < contours.size(); ++i)
   {
        cv::Rect rt = cv::boundingRect(contours[i]);
        if(rt.area() > 300)
            rectangle(img,rt,CV_RGB(255,0,0),3);
   }

     Position::PUtils::ShowImage("test",img);


   waitKey(0);

TESTEND()