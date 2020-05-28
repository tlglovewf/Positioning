/**
 *   P_Database.h
 *   
 *   add by tu li gen   2020.5.26
 * 
 */

#ifndef __PDATABASE_H_H_
#define __PDATABASE_H_H_

#include "P_Interface.h"
#include "Thirdparty/sqlite3/sqlite3.h"
namespace Position
{
    //sqlite3 数据库
    class PSqlite3Database : public IDatabase
    {
    public:
        PSqlite3Database():mpDB(NULL){}
        ~PSqlite3Database()
        {
            close();
        }
         //打开数据库
        virtual bool open(const std::string &path);
        //关闭数据库
        virtual void close();
        //执行
        virtual bool exec(const std::string &str, char **&result, int &col, int &row);
        //释放资源
        virtual void releaseDatas(char **&result);
    protected:
        sqlite3 *mpDB;
    };
};

#endif