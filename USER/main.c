#include "led.h"
#include "delay.h"
#include "stdio.h"
#include "string.h"
#include "sys.h"
#include "usart.h"

#define CSB_Wakeup() GPIO_ResetBits(GPIOA, GPIO_Pin_1)    // wakeup
#define CSB_Sleep()	 GPIO_SetBits(GPIOA, GPIO_Pin_1)      // sleep

#define Dis_Len 2
volatile u8 Distance[Dis_Len] = {0, 0};
volatile u8 Dis_Index = 0;

void CSB_init(void);	
void uart2_init(u32 bound);
void Usart2_Set(u8 sta);
u8   Measured_Range(void);    // 返回测量距离 0~255  单位：cm
	
 int main(void)
 {	
	u8 distance_cm = 0; 
	
	delay_init();	    	  // 延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	CSB_init(); 
	uart_init(9600);	      // 串口1初始化为9600
	uart2_init(9600);	      // 串口2初始化为9600

	printf("STM32F1 CSB Test\r\n");  
	while(1)
	{
		distance_cm = Measured_Range();
		if(distance_cm) printf("distance = %d cm\r\n", distance_cm);
		else            printf("Measured_Error\r\n");
		delay_ms(1000);delay_ms(1000);   
	}	 
}

void uart2_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能USART2，GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	
	//USART2_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2

	//USART2_RX	  GPIOA.3初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3  

	//USART2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART2, &USART_InitStructure);     //初始化串口2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	Usart2_Set(0);                                //关闭串口2 
}

void Usart2_Set(u8 sta)
{
	if(sta) USART_Cmd(USART2, ENABLE);                    //使能串口2 
	else    USART_Cmd(USART2, DISABLE);                   //关闭串口2	
}

void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		Distance[Dis_Index++] = USART_ReceiveData(USART2);	// 读取接收到的数据
	} 
} 


void CSB_init(void)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOA时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //PA.1
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.1

	CSB_Sleep();
}

// 返回测量距离 0~255  单位：cm
// 0：测距出错
u8 Measured_Range(void)
{
	u8 distance_cm, error_timer = 0;
	
Detectde:	
	distance_cm = 0;
	CSB_Wakeup();
	delay_ms(1);       // 至少50us   唤醒
	
	delay_ms(4);       // 系统唤醒3ms后发出触发信号0x55  
	Usart2_Set(1);     // 开启U2接收中断，准备接受测量结果
	while((USART2->SR&0X40) == 0) ;       // 直到上一次发送完毕 
	USART_SendData(USART2, 0x55); 
	
	delay_ms(25);      // 等待串口返回测量结果
	CSB_Sleep();
	Usart2_Set(0);     // 关闭U2接收中断
	
	if(Dis_Index == Dis_Len) // ok
	{
		distance_cm = ( (( (u16)Distance[0] << 8 ) + Distance[1]) / 10 ) & 0xff;    // 限定distance_cm在[0, 255]范围内
		return distance_cm;
	}
	else
	{
		if(++error_timer == 10) return 0;     // 测距出错
		//printf("error %d, Retrying...\r\n", Dis_Index);
		Distance[0] = 0;    // 清零
		Distance[1] = 0;    // 清零
		Dis_Index = 0;
		goto Detectde;
	}
}
