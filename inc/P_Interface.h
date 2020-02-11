/**
 *   p_inteface.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PINTERFACE_H_H_
#define __PINTERFACE_H_H_
#include "P_Types.h"
namespace Position
{
    // base class 
    class IBase
    {
    public:
        IBase(){}
        virtual ~IBase(){}
    };
    // data interface
    class IData : public IBase
    {
    public:
        //预处理数据
        virtual void preprocess(const std::string &imgpath, const std::string &pstpath) = 0;

        //第一个元素
        virtual FrameVIter begin() = 0;

        //最后一个元素
        virtual FrameVIter end() = 0;

        
    };


    //配置属性
    class IConfigParam : public IBase
    {
    public:
        // 获取值
        virtual void* data() = 0; 
    };
#define GETCFGVALUE(v,type) (*reinterpret_cast<type*>(v->data()))
#define SETCFGVALUE(v,d)    (GETVALUE(v,decltype(d))) = d
    // config interface
    class IConfig : public IBase
    {
    public:
        // 加载配置文件
        virtual void load(const std::string &path) = 0;

        // 获取值
        virtual IConfigParam* operator[](const std::string &name) = 0;
        
    };


    // serialization interface
    class IWriter : public IBase
    {
    public:
    };

    // visual interface
    class IViewer : public IBase
    {
    public:
    };

    //检测对象
    class IDetector : public IBase
    {
    public:

    };

    class IPositioning : public IBase
    {
    public:

    };

} // namespace Position

#endif