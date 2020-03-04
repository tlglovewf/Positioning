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
                      ImgFps(1),
                      FeatureCnt(2000),
                      PyramidLevel(8),
                      ScaleFactor(1.2),
                      SearchScale(200),
                      ImgPath(""),
                      PosPath(""),
                      OutPath(""),
                      ViewEnable(1),
                      ViewerW(0),
                      ViewerH(0),
                      ViewptX(0),
                      ViewptY(0),
                      ViewptZ(0),
                      ViewptF(0)
    {
        PUSH_MAP(StNo);
        PUSH_MAP(EdNo);
        PUSH_MAP(ImgWd);
        PUSH_MAP(ImgHg);
        PUSH_MAP(ImgFps);
        PUSH_MAP(FeatureCnt);
        PUSH_MAP(PyramidLevel);
        PUSH_MAP(SearchScale);
        PUSH_MAP(ScaleFactor);
        PUSH_MAP(ImgPath);
        PUSH_MAP(PosPath);
        PUSH_MAP(OutPath);
        PUSH_MAP(ViewEnable);
        PUSH_MAP(ViewerW);
        PUSH_MAP(ViewerH);
        PUSH_MAP(ViewptX);
        PUSH_MAP(ViewptY);
        PUSH_MAP(ViewptZ);
        PUSH_MAP(ViewptF);
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
            READ_VALUE(ImgFps);
            READ_VALUE(FeatureCnt);
            READ_VALUE(PyramidLevel);
            READ_VALUE(SearchScale);
            READ_VALUE(ScaleFactor);
            READ_VALUE(ImgPath);
            READ_VALUE(PosPath);
            READ_VALUE(OutPath);
            READ_VALUE(ViewEnable);
            READ_VALUE(ViewerW);
            READ_VALUE(ViewerH);
            READ_VALUE(ViewptX);
            READ_VALUE(ViewptY);
            READ_VALUE(ViewptZ);
            READ_VALUE(ViewptF);
            //其他信息加载
            loadmore();
        }
    }

    WeiyaConfig::WeiyaConfig(const std::string &path):
                               ExtriPath(""),
                               BsPath("")
    {
        PUSH_MAP(ExtriPath);
        PUSH_MAP(BsPath);


        load(path);
    }

    void WeiyaConfig::loadmore()
    {
        READ_VALUE(ExtriPath);
        READ_VALUE(BsPath);
    }
}