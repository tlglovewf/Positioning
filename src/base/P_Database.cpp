#include "P_Database.h"
#include "P_Checker.h"
#include "P_IOHelper.h"

namespace Position
{
     //打开数据库
    bool PSqlite3Database::open(const std::string &path)
    {
       
        if(PATHCHECK(path))
        {
            
            LOG_INFO_F("Open %s !!!",path.c_str());
            
            try
            {   
                
                int rc = sqlite3_open_v2(path.c_str(),&mpDB,SQLITE_OPEN_READONLY,NULL);
    
                if( rc != SQLITE_OK)
                {
                    LOG_CRIT_F("Open %s Failed!!",path.c_str());
                    return false;
                }
            }
            catch(const std::exception& e)
            {
                LOG_CRIT_F("Open Error : %s",e.what());
                return false;
            }
            return true;
        }
        else
        {
            // LOG_CRIT_F("%s Not Found!!!",path.c_str());
            return false;
        }
        
    }

    //关闭数据库
    void PSqlite3Database::close()
    {
        if(NULL != mpDB)
        {
            sqlite3_close(mpDB);
        }
    }
    
    //释放资源
    void PSqlite3Database::releaseDatas(char **&result) 
    {
        if((NULL != result) &&
           (NULL != mpDB))
        {
            sqlite3_free_table(result);
        }
    }

    //执行
    bool PSqlite3Database::exec(const std::string &str, char **&result, int &col, int &row)
    {
        if(NULL == mpDB)
        {
            assert(NULL);
            PROMTD_S("Database not open!!!");
        }
        char *pszErrMsg = NULL;

        int rc =  sqlite3_get_table(mpDB,str.c_str(),&result,&row,&col,&pszErrMsg);
        if((SQLITE_OK != rc) || 
           (0 == row))
        {
            LOG_CRIT_F("%s No Data!!!",str.c_str());
            if(NULL != pszErrMsg)
            {
                LOG_CRIT_F("%s error : %s",str.c_str(),pszErrMsg);
                sqlite3_free(pszErrMsg);
            }
            return false;
        }
        return true;
    }
}