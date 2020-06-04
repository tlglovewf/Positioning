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


#define MSG(X) #X


int main()
{
    cout << MSG(TEST) << endl;

    // Mat im = imread("/media/tu/Work/Datas/newdata/0-059377-531-0000301.jpg");
    
    // INITGLOBALMASK(im.size());

    
    // SETGLOBALMASK(Rect2i(0,im.rows - 250,im.cols,250));

    // cout << CHECKMASK(Point2f(500,500)) << endl;
    // cout << CHECKMASK(Point2f(500, im.rows-100)) << endl;



    return -1;
}