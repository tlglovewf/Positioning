#define TESTBEGIN()         int main(int argc, char **agrv){
#define TESTEND()           return 1;}
#define TEST_PARAM1         agrv[0]
#define TEST_PARAM2         agrv[1]
#define TEST_PARAM3         agrv[2]
#define TEST_PARAM4         agrv[3]
#define CHECKPARAMSIZE(N)   if(argc < N)
//add more param