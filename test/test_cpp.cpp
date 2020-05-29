#include "P_Interface.h"
#include "P_IOHelper.h"
#include "P_Checker.h"
#include "project/newhwproject.h"
#include "Thirdparty/sqlite3/sqlite3.h"
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

    return -1;
    sqlite3 *pDB = NULL;
    sqlite3_open("/media/tu/Work/Datas/9-200524-00/Output/NI00001-CalibrationDatabase-0512.sdb", &pDB);

    int nrow = 0;
    int ncol = 0;
    char *perror = NULL;
    char **result = NULL;

    int ret = sqlite3_get_table(pDB, "select * from TypeCCameraInternal", &result, &nrow, &ncol, &perror);

    if (ret == SQLITE_OK)
    {
        for (size_t i = 0; i < nrow; ++i)
        {
            cout << result[ncol++] << endl;
            cout << result[ncol++] << endl;
            cout << result[ncol++] << endl;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
            cout << result[ncol++] << endl;
            cout << result[ncol++] << endl;
            cout << result[ncol++] << endl;
            cout << result[ncol++] << endl;
            cout << result[ncol++] << endl;
        }
    }

    sqlite3_close(pDB);
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