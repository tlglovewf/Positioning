#include "P_Config.h"
#include "P_IOHelper.h"
#include "P_Checker.h"
namespace Position
{
     static std::shared_ptr<IConfig> g_globaleConfig;

    //获取单例
    std::shared_ptr<IConfig>& IConfig::Instance()
    {
        assert(g_globaleConfig);
        return g_globaleConfig;
    }

    //设置单例
    void IConfig::SetInstance(const std::shared_ptr<IConfig> &pinstance)
    {
        g_globaleConfig = pinstance;
    }
    //构造
    PConfig::PConfig():
                      StNo(0), 
                      EdNo(0),
                      ImgWd(0),  
                      ImgHg(0),
                      ImgFps(1),
                    //   ImgRgb(1),
                      FeatureCnt(2000),
                      PyramidLevel(8),
                      ScaleFactor(1.2),
                    //   MatchRatio(0.8),
                      SearchRadius(200),
                      LogPath(""),
                      LogPrio("DEBUG"),
                      ImgPath(""),
                      SemPath(""),
                      PosPath(""),
                      OutPath(""),
                      ViewEnable(1),
                    //   ViewerW(0),
                    //   ViewerH(0),
                    //   ViewptX(0),
                    //   ViewptY(0),
                    //   ViewptZ(0),
                    //   ViewptF(0),
                      MapSave(0),
                      InitializationMode(0),
                      InitImgLength(0),
                      MaskEnable(0),
                      Lx(-1),
                      Ly(-1),
                      Wd(0),
                      Ht(0)
    {
        PUSH_MAP(StNo);
        PUSH_MAP(EdNo);
        PUSH_MAP(ImgWd);
        PUSH_MAP(ImgHg);
        PUSH_MAP(ImgFps);
        // PUSH_MAP(ImgRgb);
        PUSH_MAP(FeatureCnt);
        PUSH_MAP(PyramidLevel);
        PUSH_MAP(SearchRadius);
        PUSH_MAP(ScaleFactor);
        // PUSH_MAP(MatchRatio);
        PUSH_MAP(LogPath);
        PUSH_MAP(LogPrio);
        PUSH_MAP(ImgPath);
        PUSH_MAP(SemPath);
        PUSH_MAP(PosPath);
        PUSH_MAP(OutPath);
        PUSH_MAP(ViewEnable);
        // PUSH_MAP(ViewerW);
        // PUSH_MAP(ViewerH);
        // PUSH_MAP(ViewptX);
        // PUSH_MAP(ViewptY);
        // PUSH_MAP(ViewptZ);
        // PUSH_MAP(ViewptF);
        PUSH_MAP(MapSave);
        PUSH_MAP(InitializationMode);
        PUSH_MAP(InitImgLength);
        PUSH_MAP(MaskEnable);
        PUSH_MAP(Lx);
        PUSH_MAP(Ly);
        PUSH_MAP(Wd);
        PUSH_MAP(Ht);
    }

    //析构
    PConfig::~PConfig()
    {
    }

    // 加载配置文件
    void PConfig::load(const std::string &path)
    {
        if(!PATHCHECK(path))
        {
            PROMT_V(path.c_str()," Config Path Not Found..");
            return;
        }
        else
        {
            if(!mSettings.isOpened())
            {
                if(!mSettings.open(path,FileStorage::READ))
                {//打开失败
                    PROMT_V(path.c_str(), "Open Error.");
                    exit(-1);
                }
            }

            //配置文件加载成功 开始读取参数
            READ_VALUE(StNo);
            READ_VALUE(EdNo);
            READ_VALUE(ImgWd);
            READ_VALUE(ImgHg);
            READ_VALUE(ImgFps);
            // READ_VALUE(ImgRgb);
            READ_VALUE(FeatureCnt);
            READ_VALUE(PyramidLevel);
            READ_VALUE(SearchRadius);
            READ_VALUE(ScaleFactor);
            // READ_VALUE(MatchRatio);
            READ_VALUE(LogPath);
            READ_VALUE(LogPrio);
            READ_VALUE(ImgPath);
            READ_VALUE(SemPath);
            READ_VALUE(PosPath);
            READ_VALUE(OutPath);
            READ_VALUE(ViewEnable);
            // READ_VALUE(ViewerW);
            // READ_VALUE(ViewerH);
            // READ_VALUE(ViewptX);
            // READ_VALUE(ViewptY);
            // READ_VALUE(ViewptZ);
            // READ_VALUE(ViewptF);
            READ_VALUE(MapSave);
            READ_VALUE(InitializationMode);
            READ_VALUE(InitImgLength);
            READ_VALUE(MaskEnable);
            READ_VALUE(Lx);
            READ_VALUE(Ly);
            READ_VALUE(Wd);
            READ_VALUE(Ht);
            //其他信息加载
            loadmore();
        }
    }
}