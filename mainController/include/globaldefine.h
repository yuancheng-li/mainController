#ifndef __GLOBALDEFINE_H__
#define __GLOBALDEFINE_H__

// 定义机器人类型
#define xMateErPro 

#ifdef xMateErPro
	constexpr auto m_DoF = 7;
#endif // xMateErPro




// 定义程序中的输出类型
#define __ERROR__
#define __WARNING__
#define __PRINT__
#define __RESULT__

// 打印运行错误数据
#ifdef __ERROR__
#define IERROR(format, ...)  printf("\033[7m\033[1m\033[31m[ERROR]\033[0m FILE: [ %s %d ] \n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
#define IERROR(...)
#endif

// 打印关键程序警告信息
#ifdef __WARNING__
#define IWARNING(format, ...)  printf("\033[7m\033[1m\033[33m[WARNING]\033[0m FILE: [ %s %d ] \n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
#define IWARNING(...)
#endif
// \033[0m
// 打印程序运行过程输出
#ifdef __PRINT__
#define IPRINT(format, ...)  printf("\033[7m\033[1m\033[37m[PRINT]\033[0m  FILE: [ %s %d ]\n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
#define IPRINT(...)
#endif
// 打印一些程序结果
#ifdef __RESULT__
#define IRESULT(format, ...)  printf("\033[7m\033[1m\033[32m[RESULT]\033[0m FILE: [ %s %d ]\n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
#define IRESULT(...)
#endif


#endif
