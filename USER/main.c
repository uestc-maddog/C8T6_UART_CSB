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
u8   Measured_Range(void);    // ���ز������� 0~255  ��λ��cm
	
 int main(void)
 {	
	u8 distance_cm = 0; 
	
	delay_init();	    	  // ��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	CSB_init(); 
	uart_init(9600);	      // ����1��ʼ��Ϊ9600
	uart2_init(9600);	      // ����2��ʼ��Ϊ9600

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
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART2��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	
	//USART2_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2

	//USART2_RX	  GPIOA.3��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3  

	//USART2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(USART2, &USART_InitStructure);     //��ʼ������2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	Usart2_Set(0);                                //�رմ���2 
}

void Usart2_Set(u8 sta)
{
	if(sta) USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2 
	else    USART_Cmd(USART2, DISABLE);                   //�رմ���2	
}

void USART2_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
	{
		Distance[Dis_Index++] = USART_ReceiveData(USART2);	// ��ȡ���յ�������
	} 
} 


void CSB_init(void)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOAʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //PA.1
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.1

	CSB_Sleep();
}

// ���ز������� 0~255  ��λ��cm
// 0��������
u8 Measured_Range(void)
{
	u8 distance_cm, error_timer = 0;
	
Detectde:	
	distance_cm = 0;
	CSB_Wakeup();
	delay_ms(1);       // ����50us   ����
	
	delay_ms(4);       // ϵͳ����3ms�󷢳������ź�0x55  
	Usart2_Set(1);     // ����U2�����жϣ�׼�����ܲ������
	while((USART2->SR&0X40) == 0) ;       // ֱ����һ�η������ 
	USART_SendData(USART2, 0x55); 
	
	delay_ms(25);      // �ȴ����ڷ��ز������
	CSB_Sleep();
	Usart2_Set(0);     // �ر�U2�����ж�
	
	if(Dis_Index == Dis_Len) // ok
	{
		distance_cm = ( (( (u16)Distance[0] << 8 ) + Distance[1]) / 10 ) & 0xff;    // �޶�distance_cm��[0, 255]��Χ��
		return distance_cm;
	}
	else
	{
		if(++error_timer == 10) return 0;     // ������
		//printf("error %d, Retrying...\r\n", Dis_Index);
		Distance[0] = 0;    // ����
		Distance[1] = 0;    // ����
		Dis_Index = 0;
		goto Detectde;
	}
}
