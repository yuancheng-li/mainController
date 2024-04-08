#ifndef __GLOBALDEFINE_H__
#define __GLOBALDEFINE_H__

// �������������
#define xMateErPro 

#ifdef xMateErPro
	constexpr auto m_DoF = 7;
#endif // xMateErPro




// ��������е��������
#define __ERROR__
#define __WARNING__
#define __PRINT__
#define __RESULT__

// ��ӡ���д�������
#ifdef __ERROR__
#define IERROR(format, ...)  printf("\033[7m\033[1m\033[31m[ERROR]\033[0m FILE: [ %s %d ] \n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
#define IERROR(...)
#endif

// ��ӡ�ؼ����򾯸���Ϣ
#ifdef __WARNING__
#define IWARNING(format, ...)  printf("\033[7m\033[1m\033[33m[WARNING]\033[0m FILE: [ %s %d ] \n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
#define IWARNING(...)
#endif
// \033[0m
// ��ӡ�������й������
#ifdef __PRINT__
#define IPRINT(format, ...)  printf("\033[7m\033[1m\033[37m[PRINT]\033[0m  FILE: [ %s %d ]\n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
#define IPRINT(...)
#endif
// ��ӡһЩ������
#ifdef __RESULT__
#define IRESULT(format, ...)  printf("\033[7m\033[1m\033[32m[RESULT]\033[0m FILE: [ %s %d ]\n" format, __FILE__, __LINE__, ##__VA_ARGS__  )
#else
#define IRESULT(...)
#endif


#endif
