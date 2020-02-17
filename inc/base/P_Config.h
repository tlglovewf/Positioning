/**
 *   P_Config.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PCONFIG_H_H_
#define __PCONFIG_H_H_
#include "P_Interface.h"
#include <map>
namespace Position
{

    //配置属性
    template<typename T>
    class ConfigParam  : public IConfigParam
    {
    public:
        ConfigParam(){}
        ConfigParam(T v):_v(v){}
        //函数操作符重载
        T& operator()()
        {
            return _v;
        }

        // 获取值
        virtual void* data()
        {
            return reinterpret_cast<void*>(&_v);
        }

        //从文件读取
        virtual void read(const FileStorage &file,const string &nm) = 0;
        
    protected:
       T _v;
    };

#define DECLARECONS(class_name,type) class_name(type t):ConfigParam<type>(t){}

    //int 类型 配置参数
    class IntConfigParam : public ConfigParam<int>
    {
    public:
        DECLARECONS(IntConfigParam,int)

        //从文件读取
        virtual void read(const FileStorage &file,const string &nm) 
        {
            _v = file[nm];
        }
    };

    //float 类型
    class FloatConfigParam : public ConfigParam<float>
    {
    public:
        DECLARECONS(FloatConfigParam,float)

        //从文件读取
        virtual void read(const FileStorage &file,const string &nm) 
        {
            _v = file[nm];
        }
    };

    //string 类型 配置参数
    class StringConfigParam : public ConfigParam<string>
    {
    public:
        DECLARECONS(StringConfigParam,string)

          //从文件读取
        virtual void read(const FileStorage &file,const std::string &nm)
        {
            cv::String v;
            file[nm.c_str()] >> v;
            _v = v;
        }
    };

    //mat 类型 配置参数
    class MatConfigParam : public ConfigParam<Mat>
    {
    public:
        DECLARECONS(MatConfigParam,Mat)

        //从文件读取
        virtual void read(const FileStorage &file,const string &nm) 
        {
            file[nm] >> _v;
        }
    };

    //add more param types


    /**********************************
     **********************************
     **********************************/

    //配置实现类
    class PConfig : public IConfig
    {
    public:
        typedef std::map<std::string, IConfigParam*> ParamsMap;
        typedef ParamsMap::iterator                  ParamsMapIter;

        //构造
        PConfig();
        //析构
        ~PConfig();
        // 加载配置文件
        virtual void load(const std::string &path);

        // 获取值
        virtual IConfigParam* operator[](const std::string &name)
        {
            return mConfigParams[name];
        }
    protected:

        //其他信息加载
        virtual void loadmore() {}

    protected:
        ParamsMap           mConfigParams;

        FileStorage         mSettings;

        //define config params
        IntConfigParam      StNo;           //开始序号
        IntConfigParam      EdNo;           //结束序号

         //相机参数     
        IntConfigParam      ImgWd;          //图像宽度
        IntConfigParam      ImgHg;          //图像长度

        IntConfigParam      FeatureCnt;     //特征点数量
        IntConfigParam      PyramidLevel;   //金字塔层数

        FloatConfigParam    ScaleFactor;    //金字塔缩放比例

        //路径参数
        StringConfigParam   ImgPath;        //图片路径
        StringConfigParam   PosPath;        //数据路径
        StringConfigParam   OutPath;        //输出路径

        //add more ...

    };

    //维亚配置数据
    class WeiyaConfig : public PConfig
    {
    public:
        WeiyaConfig();
    protected:
        //其他信息加载
        virtual void loadmore();
    protected:

        StringConfigParam ExtriPath;  //相机参数文件路径
        StringConfigParam BsPath;     //安置参数文件路径
    };
}

#endif