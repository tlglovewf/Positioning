#include "P_Interface.h"
#include "P_Writer.h"
#include "P_Checker.h"
// #include "log4cpp/Category.hh"
// #include "log4cpp/PatternLayout.hh"
// #include "log4cpp/OstreamAppender.hh"
// #include <log4cpp/Appender.hh>
// #include <log4cpp/FileAppender.hh>
// #include <log4cpp/Priority.hh>
// #include <log4cpp/RollingFileAppender.hh>
// #include <log4cpp/PropertyConfigurator.hh>
template<typename T>
inline void display(T items)
{
    for(auto i : items)
    {
        cout << i << " ";
    }
    cout << endl;
}

#define _TOSTR(a) #a
#define NMB_STR(a) _TOSTR(a)
#define FILE_LOG() "{" __FILE__ "}" 
#define LINE_LOG() "{" NMB_STR(__LINE__) "}"
#define PRINTMESSAGE(X) FILE_LOG()LINE_LOG()":" X

#define FM_T_V "%d" "%s"
#define FM_T_S "%s"
int main()
{
    LOG_INITIALIZE("");// "logsetting.conf");
    cout << "1" << endl;
    // LOG_PROMOT()crit("test");
    // Position::PLogManager::s_Mgr->instacne().error("test");

    LOG_INFO("PoseEstimate ...");

    // LOG_PROMT()info(FM_T_V,10,"test");
    // LOG_PROMT()warn("hello");
    cout << FM_T_V << endl;

    // cout << FILE_LOG() << endl;
    // cout << LINE_LOG() << endl;
    // try
    // {
    //    log4cpp::PropertyConfigurator::configure("./logsetting.conf");
    // }
    // catch(const std::exception& e)
    // {
    //     std::cerr << e.what() << '\n';
    // }
    
    // log4cpp::Category::getRoot().error("hello world");

    // return 0;

    // log4cpp::PatternLayout* pLayerout = new log4cpp::PatternLayout();
    // pLayerout->setConversionPattern("{%d{%Y-%m-%d %H:%M:%S.%l}}{%p}: %m%n"); // {time}{loglevel}{file}{line}:message

    // // log4cpp::Appender* fileAppender = new log4cpp::FileAppender("fileAppender", "wxb.log");//创建一个Appender;
    // // fileAppender->setLayout(pLayerout);//将指定的Layout添加到Appender;

    // log4cpp::RollingFileAppender* rollfileAppender = new log4cpp::RollingFileAppender(
    //     "rollfileAppender", "rollwxb.log", 1 * 1024, 1); // 超过5k自动回滚，最大文件数为1
    // rollfileAppender->setLayout(pLayerout);

    // log4cpp::PatternLayout* pLayerout1 = new log4cpp::PatternLayout();
    // pLayerout1->setConversionPattern("{%d{%Y-%m-%d %H:%M:%S.%l}}{%p}%m%n"); // {time}{loglevel}{file}{line}:message

    // log4cpp::OstreamAppender app("osAppender", &cout);
    // app.setLayout(pLayerout1);
    
    // log4cpp::Category& root = log4cpp::Category::getRoot();//从系统中得到Category的根;
    // // root.addAppender(fileAppender);
    // root.addAppender(app);
    // root.addAppender(rollfileAppender);
    // root.setPriority(log4cpp::Priority::WARN);

    // for(int i = 0; i < 3; ++i)
    // {
    //     root.warn(PRINTMESSAGE("test"));
    //     root.crit
    //     root.error(PRINTMESSAGE("error"));
    // }

    // log4cpp::PropertyConfigurator::

    // log4cpp::Category::shutdown();

    // vector<int> vts;jju
    // vts.emplace_back(1);
    // vts.emplace_back(2);
    // vts.emplace_back(3);

    // vector<int*> vt;
    // std::transform(vts.begin(),vts.end(),back_inserter(vt),[](int &a)->int*
    // {
    //     return &a;
    // });

    // display(vt);
    // *vt[0] = 10;
    // display(vts);
  
    // cout << distance(vts.begin(),vts.end()) << endl;

    // int x[] = {3,4,5,1,2};

    // std::copy(x,x+5,back_inserter(vts));
    
    // display(vts);
    // std::sort(vts.begin(),vts.end());
    // vts.erase(std::unique(vts.begin(),vts.end()),vts.end());
    // display(vts);

    return -1;
}