#include "P_IOHelper.h"
#include "P_Converter.h"
#include "P_Utils.h"
#include "P_Frame.h"
#include "P_Checker.h"

//log4cpp
#include "log4cpp/Category.hh"
#include "log4cpp/PatternLayout.hh"
#include "log4cpp/OstreamAppender.hh"
#include "log4cpp/FileAppender.hh"
#include "log4cpp/PropertyConfigurator.hh"

namespace Position
{
#define HEADTRACSTR "Frame Trace"
#define HEADMPSTR   "Map Points"

#define BEGINFILEREGION(PATH,MD)  try                               \
                                  {                                 \
                                     if(open(PATH,std::ios::MD))    \
                                     {

#define ENDFILEREGION()               mfile.close();                \
                                     }                               \
                                  }                                 \
                                  catch(const std::exception& e)    \
                                  {                                 \
                                      std::cerr << e.what() << '\n';\
                                  }


    void DefaultWRNode::writeItem(const std::string &name, const cv::Mat &pose)
    {
        std::string str = PConverter::toString(pose);
        if(!name.empty())
        {
            str = name + ":" + str;
        }
        mfile << str << endl;
    }

    //加载地图轨迹
    void PMapTraceSer::loadMap(const std::string &path) 
    {
        assert(mpMap);
        
        if(!PATHCHECK(path))
        {
            LOG_ERROR("File Path error.Please check map file path.");
            return;
        }


        BEGINFILEREGION(path,in)

        string head;
        getline(mfile,head);
        if(head == HEADTRACSTR)
        {
            LOG_INFO("Load Trace File ...");
            while(!mfile.eof())
            {//遍历文件 创建关键帧
                string line;
                getline(mfile,line);
                if(line.empty())
                    continue;
                StringVector svs = PUtils::SplitString(line,":");
                assert(svs.size() == 2);
                FrameData *data = new FrameData();
                data->_name = svs[0];
                IFrame *pf = new PFrame(data,mpMap->frameCount());
                pf->setPose(PConverter::str2CVMat(svs[1]));
                mpMap->createKeyFrame(pf);
            }
            LOG_INFO("Trace File finished.");
        }
        else
        {
            LOG_ERROR("It's not a avilable trace file.");
        }
        ENDFILEREGION()
    }

    //保存地图轨迹
    void PMapTraceSer::saveMap(const std::string &path) 
    {
        assert(mpMap);
        assert(!path.empty());
        BEGINFILEREGION(path,out)

        KeyFrameVector keyfms = std::move(mpMap->getAllFrames());
        if(keyfms.empty())
            return;
        
        //write head 
        mfile << HEADTRACSTR << endl;
        for(size_t i = 0; i < keyfms.size(); ++i)
        {
            const std::string &name = keyfms[i]->getData()->_name;
            const Mat& pose = keyfms[i]->getPose();
            writeItem(name,pose);
        }

        ENDFILEREGION()
    }

     //加载地图轨迹
    void PMapPointsSer::loadMap(const std::string &path) 
    {
        assert(mpMap);
        assert(!path.empty());
        BEGINFILEREGION(path,in)

        string head;
        getline(mfile,head);
        if(head == HEADMPSTR)
        {
            LOG_INFO("Load Map ponits ...");
            while(!mfile.eof())
            {//遍历文件 创建关键帧
                string line;
                getline(mfile,line);
                if(line.empty())
                    continue;

                cv::Mat pt = PConverter::str2CVMat(line,true);
                if(pt.empty())
                    continue;
                mpMap->createMapPoint(pt);
            }
            LOG_INFO("Map points load finished.");
        }
        else
        {
            PROMT_S("It's not a avilable mappoints file")
        }
        

        ENDFILEREGION()
    }

    //保存地图轨迹
    void PMapPointsSer::saveMap(const std::string &path) 
    {
        assert(mpMap);
        assert(!path.empty());
        BEGINFILEREGION(path,out)

        MapPtVector pts = std::move(mpMap->getAllMapPts());
  
        if(pts.empty())
            return;
        //write head 
        mfile << HEADMPSTR << endl;
        for(size_t i = 0; i < pts.size(); ++i)
        {
            const Mat &pt = pts[i]->getWorldPos();
            writeItem("",pt);
        }

        ENDFILEREGION()
    }


    PLogManager* PLogManager::s_Mgr = NULL;

    class PriorityFilter : public log4cpp::Filter
    {
    protected:
        virtual Decision _decide(const log4cpp::LoggingEvent& event) 
        {
            if(event.priority < log4cpp::Priority::DEBUG)
            {
                return log4cpp::Filter::ACCEPT;
            }   
            else
            {
                return log4cpp::Filter::DENY;
            }
        }
    };

    //从log配置文件加载
    PLogManager::PLogManager(const std::string &settingPath)
    {
        if(PATHCHECK(settingPath))
        {
            try
            {
                PROMT_S("Load logger setting file.");
                log4cpp::PropertyConfigurator::configure(settingPath);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            } 
        }
        else
        {//配置文件不准就默认生成一个log文件
            PROMT_S("Log setting file not exist.Create deafult logger.");
             //工程配置中 只能设置log文件输出信息 其他所有设置使用默认
            // {time}{loglevel}{file}{line}:message
            const std::string patternformat = "{%d{%Y-%m-%d %H:%M:%S}}{%p}: %m%n";
    
            log4cpp::PatternLayout* consleLayerout = new log4cpp::PatternLayout();
            consleLayerout->setConversionPattern(patternformat); 
            log4cpp::OstreamAppender *consoleLogger = new log4cpp::OstreamAppender("ConsoleLogger", &cout);
            consoleLogger->setLayout(consleLayerout);
    
            log4cpp::PatternLayout* filelayout = new log4cpp::PatternLayout();
            filelayout->setConversionPattern(patternformat);
    
            string path = string(_DATE_LOG_()) + ".log";// "Position.log";
            log4cpp::Appender* fileLogger = new log4cpp::FileAppender("FileLogger", path,false);
            fileLogger->setLayout(filelayout);
            fileLogger->setFilter(new PriorityFilter());
            log4cpp::Category& root = log4cpp::Category::getRoot();//从系统中得到Category的根;

            root.addAppender(consoleLogger);
            root.addAppender(fileLogger);
        }
    }
    //从工程配置加载
    PLogManager::PLogManager(const shared_ptr<IConfig> &pcfg):
    PLogManager(GETCFGVALUE(pcfg,LogPath,string))
    {
        std::string prior = GETCFGVALUE(pcfg,LogPrio,string);
        log4cpp::Priority::Value ePri ;
        if(prior == "DEBUG")
        {
            ePri = log4cpp::Priority::DEBUG;
        }
        else if(prior == "INFO")
        {
            ePri = log4cpp::Priority::INFO;
        }
        else if(prior == "WARNNING")
        {
            ePri = log4cpp::Priority::WARN;
        }
        else if(prior == "ERROR")
        {
            ePri = log4cpp::Priority::ERROR;
        }
        else if(prior == "CRITICAL")
        {
            ePri = log4cpp::Priority::CRIT;
        }
        else
        {
            ePri = log4cpp::Priority::DEBUG;
        }
        PROMT_V("Set Logger Priority",prior);
        log4cpp::Category::getRoot().setPriority(ePri); 
    }
    PLogManager::~PLogManager()
    {
        log4cpp::Category::shutdown();
    }
    //获取log 输出类, 默认就是root
    log4cpp::Category& PLogManager::instacne(const std::string& str /*= ""*/)
    {
        if(str.empty())
        {
            log4cpp::Category& root = log4cpp::Category::getRoot();
            return root;
        }
        else
        {
            return log4cpp::Category::getRoot().getInstance(str);
        }
    }
}