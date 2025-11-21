#include "printf.h"
#include "usart.h"

//取消ARM的半主机工作模式
#pragma import(__use_no_semihosting)                             
struct __FILE
{ 
	int handle; 
}; 

FILE __stdout;          
void _sys_exit(int x) 
{ 
	x = x; 
}

int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0){}
    USART1->DR = (uint8_t) ch;      
	return ch;
}
