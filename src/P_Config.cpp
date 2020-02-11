#include "P_Config.h"
#include "P_Writer.h"
namespace Position
{
#define PUSH_MAP(V)   mConfigParams.insert(std::make_pair(#V,&V))
#define READ_VALUE(V) V.read(mSettings,#V)
#define RELEASE(obj) if(obj){delete obj;obj = NULL;}
    //构造
    PConfig::PConfig():
                      StNo(0), 
                      EdNo(0),
                      ImgPath(""),
                      PosPath(""),
                      OutPath(""),
                      ImgWd(0),  
                      ImgHg(0)
    {
        PUSH_MAP(StNo);
        PUSH_MAP(EdNo);
        PUSH_MAP(ImgPath);
        PUSH_MAP(PosPath);
        PUSH_MAP(OutPath);
        PUSH_MAP(ImgWd);
        PUSH_MAP(ImgHg);
    }

    //析构
    PConfig::~PConfig()
    {
    }

    // 加载配置文件
    void PConfig::load(const std::string &path)
    {
        if(path.empty())
        {
            return;
        }
        else
        {
            if(!mSettings.isOpened())
            {
                if(!mSettings.open(path,FileStorage::READ))
                {//打开失败
                    PROMT_V("open config error",path.c_str());
                    exit(-1);
                }
            }

            //配置文件加载成功 开始读取参数
            READ_VALUE(StNo);
            READ_VALUE(EdNo);
            READ_VALUE(ImgPath);      
            READ_VALUE(PosPath);
            READ_VALUE(OutPath);
            READ_VALUE(ImgWd);
            READ_VALUE(ImgHg);
            //其他信息加载
            loadmore();
        }
    }
}