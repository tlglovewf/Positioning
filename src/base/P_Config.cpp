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
                      ImgWd(0),  
                      ImgHg(0),
                      FeatureCnt(2000),
                      PyramidLevel(8),
                      ScaleFactor(1.2),
                      SearchScale(200),
                      ImgPath(""),
                      PosPath(""),
                      OutPath("")
    {
        PUSH_MAP(StNo);
        PUSH_MAP(EdNo);
        PUSH_MAP(ImgWd);
        PUSH_MAP(ImgHg);
        PUSH_MAP(FeatureCnt);
        PUSH_MAP(PyramidLevel);
        PUSH_MAP(SearchScale);
        PUSH_MAP(ScaleFactor);
        PUSH_MAP(ImgPath);
        PUSH_MAP(PosPath);
        PUSH_MAP(OutPath);
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
            READ_VALUE(ImgWd);
            READ_VALUE(ImgHg);
            READ_VALUE(FeatureCnt);
            READ_VALUE(PyramidLevel);
            READ_VALUE(SearchScale);
            READ_VALUE(ScaleFactor);
            READ_VALUE(ImgPath);
            READ_VALUE(PosPath);
            READ_VALUE(OutPath);
           
            //其他信息加载
            loadmore();
        }
    }

    WeiyaConfig::WeiyaConfig():PConfig(),
                               ExtriPath(""),
                               BsPath("")
    {
        PUSH_MAP(ExtriPath);
        PUSH_MAP(BsPath);
    }

    void WeiyaConfig::loadmore()
    {
        READ_VALUE(ExtriPath);
        READ_VALUE(BsPath);
    }
}