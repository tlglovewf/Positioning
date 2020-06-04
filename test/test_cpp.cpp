#include "P_Interface.h"
#include "P_IOHelper.h"
#include "P_Checker.h"
#include "P_CoorTrans.h"
#include "P_Utils.h"
#include "project/newhwproject.h"
#include "Thirdparty/sqlite3/sqlite3.h"
#include "P_SemanticGraph.h"
#include "P_Mask.h"
// #include "log4cpp/Category.hh"
// #include "log4cpp/PatternLayout.hh"
// #include "log4cpp/OstreamAppender.hh"
// #include <log4cpp/Appender.hh>
// #include <log4cpp/FileAppender.hh>
// #include <log4cpp/Priority.hh>
// #include <log4cpp/RollingFileAppender.hh>
// #include <log4cpp/PropertyConfigurator.hh>
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
// typedef

// int (*callback)(void *, int, char **, char **)

int main()
{

    Mat im = imread("/media/tu/Work/Datas/newdata/0-059377-531-0000301.jpg");
    
    INITGLOBALMASK(im.size());

    
    SETGLOBALMASK(Rect2i(0,im.rows - 250,im.cols,250));

    cout << CHECKMASK(Point2f(500,500)) << endl;
    cout << CHECKMASK(Point2f(500, im.rows-100)) << endl;

    // const int catlen = 250;
    // im(Rect2i(0,im.rows - catlen,im.cols,catlen)).setTo(0);
    // imwrite("/media/tu/Work/Datas/newdata/2.jpg",im);

    // ifstream ifile("/media/tu/Work/Datas/1014-0-08C00001-200524/RawData/IMUData/1014008C00001200524.imu");

    // string line;
    //   // get length of file:
    // // ifile.seekg (0, ios::end);
    // // size_t length = ifile.tellg();
    // // cout << length << endl;
    // // ifile.seekg (0, ios::beg);

    // // ifile >> ch8 ;

    // // cout << ch8 << endl;

    // ifile.close();
    // return 0;

    // cout << FM_T_V << endl;

    return -1;
}