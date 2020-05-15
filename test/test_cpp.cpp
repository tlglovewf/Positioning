#include "P_Interface.h"
#include "log4cpp/Category.hh"
#include "log4cpp/PatternLayout.hh"
#include "log4cpp/OstreamAppender.hh"
#include <log4cpp/Appender.hh>
#include <log4cpp/FileAppender.hh>
#include <log4cpp/Priority.hh>
#include <log4cpp/RollingFileAppender.hh>
#include <log4cpp/PropertyConfigurator.hh>
template<typename T>
inline void display(T items)
{
    for(auto i : items)
    {
        cout << i << " ";
    }
    cout << endl;
}

// #define STR_INFO (string(__FILE__) + ":" + string(__FUNCTION__) + ":" + string(__LINE__))
//宏定义

int main()
{

    log4cpp::PatternLayout* pLayerout = new log4cpp::PatternLayout();
    pLayerout->setConversionPattern("{%d{%Y-%m-%d %H:%M:%S.%l}}{%p}{%F}{%L}: %m%n"); // {time}{loglevel}{file}{line}:message

    // log4cpp::Appender* fileAppender = new log4cpp::FileAppender("fileAppender", "wxb.log");//创建一个Appender;
    // fileAppender->setLayout(pLayerout);//将指定的Layout添加到Appender;

    log4cpp::RollingFileAppender* rollfileAppender = new log4cpp::RollingFileAppender(
        "rollfileAppender", "rollwxb.log", 1 * 1024, 1); // 超过5k自动回滚，最大文件数为1
    rollfileAppender->setLayout(pLayerout);

    log4cpp::PatternLayout* pLayerout1 = new log4cpp::PatternLayout();
    pLayerout1->setConversionPattern("{%d{%Y-%m-%d %H:%M:%S.%l}}{%p}{%F}{%L}: %m%n"); // {time}{loglevel}{file}{line}:message

    log4cpp::OstreamAppender app("osAppender", &cout);
    app.setLayout(pLayerout1);

    log4cpp::Category& root = log4cpp::Category::getRoot();//从系统中得到Category的根;
    // root.addAppender(fileAppender);
    root.addAppender(app);
    root.addAppender(rollfileAppender);
    root.setPriority(log4cpp::Priority::ERROR);

    for(int i = 0; i < 100; ++i)
    {
        root.warn("test");
        root.error("error");
    }

    // log4cpp::PropertyConfigurator::

    log4cpp::Category::shutdown();

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