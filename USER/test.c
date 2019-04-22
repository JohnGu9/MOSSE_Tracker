#include "myTrack.h"
#include "sys.h"
#include "delay.h"  
#include "usart.h"   
#include "led.h"
#include "lcd.h"
#include "key.h"  
#include "timer.h" 
#include "ov2640.h" 
#include "dcmi.h"
#include "sram.h"  
#include "malloc.h"
#include "arm_math.h" 

#define offset 6/10

int main(void)
{
	FSMC_SRAM_Init();				//��ʼ���ⲿSRAM  	
	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
	my_mem_init(SRAMEX);		//��ʼ���ⲿ�ڴ��
	my_mem_init(SRAMCCM);		//��ʼ��CCM�ڴ��	
	Stm32_Clock_Init(378,8,2,7);//����ʱ��,182Mhz 
	delay_init(189);			//��ʱ��ʼ��  
	uart_init(84,115200);		//��ʼ�����ڲ�����Ϊ115200 
	LED_Init();					//��ʼ��LED 
 	LCD_Init();					//LCD��ʼ��  
 	KEY_Init();					//������ʼ�� 
 	POINT_COLOR=RED;//��������Ϊ��ɫ 
	
	MOSSE_Tracker.init(	lcddev.width/2 - Window_width*Cell_side/2, 
											lcddev.height/2 - Window_height*Cell_side/2,
											Window_width,
											Window_height,
											Cell_side);	//Tracker Initialization

	LCD_ShowString(30,50,600,font_size,font_size,"Explorer STM32F4");	
	LCD_ShowString(30,50+30,600,font_size,font_size,"MOSSE_Track for Embedded Systems");	
	LCD_ShowString(30,50+30*2,600,font_size,font_size,"johngumaster@outlook.com");
	LCD_ShowString(30,50+30*3,600,font_size,font_size,"2019/3/29");
	
	while(OV2640_Init())//��ʼ��OV2640
	{
		LCD_ShowString(30,50+30*4,600,font_size,font_size,"OV2640 ERROR");
		delay_ms(200);
	    LCD_Fill(30,130,239,170,WHITE);
		delay_ms(200);
		LED0=!LED0;
	} 
	LCD_ShowString(30,50+30*4,600,font_size,font_size,"OV2640 OK");
	delay_ms(1000);
	rgb565(); 
}

inline static void rgb565(void)
{ 
	static u8 key;
	static u16 default_local[2];
	static int current_local[2];

	static u8 track_flag;
	static u8 Hi_available;
	track_flag = 0;
	Hi_available = 0;

	default_local[0] = lcddev.width/2 - Window_width*Cell_side/2;
	default_local[1] = lcddev.height/2 - Window_height*Cell_side/2;
	
	LCD_Clear(WHITE);
  POINT_COLOR=YELLOW; //change window color 
	OV2640_RGB565_Mode();	//RGB565ģʽ
	DCMI_Init();			//DCMI����
	DCMI_DMA_Init((u32)&LCD->LCD_RAM,1,1,0);//DCMI DMA���� 
#ifndef Horizon
//vertical
	OV2640_ImageWin_Set((1600-1200*lcddev.width/lcddev.height)/2,0,1200*lcddev.width/lcddev.height,1200);//��ʵ�ߴ�
#else
//horizon 	
	OV2640_ImageWin_Set(0,(1200-1600*lcddev.width/lcddev.height)/2,1600,1600*lcddev.width/lcddev.height);//��ʵ�ߴ�
#endif
	OV2640_OutSize_Set(lcddev.width,lcddev.height);
	OV2640_Contrast(3);//Increase contrast a bit 
	DCMI_Start(); 		//��������
	delay_ms(200); 
	while(1)
	{ 
		key=KEY_Scan(0); 
		if(key)
		{ 
			POINT_COLOR = YELLOW; //change window color	
			DCMI_Stop(); //ֹͣ��ʾ
			switch(key)
			{				    
				case KEY0_PRES:	//�Աȶ�����
					
					break;
				case KEY1_PRES:	//update fliter
					if(track_flag)
					{
						track_flag = 0;
						MOSSE_Tracker.window_set(default_local[0], default_local[1]);
					} else {
						MOSSE_Tracker.train(Rect);
						Hi_available = 1;
					}
					break;
				case KEY2_PRES:	//��Ч����				 

					break;
				case WKUP_PRES:	//start tracker 
					if(Hi_available)
					{
						track_flag = !track_flag;
						MOSSE_Tracker.window_set(default_local[0], default_local[1]);
					}	else
					{
						LCD_ShowString(30,50,420,16,16,"Filter not yet generated! ");	
						delay_ms(500);						
					}
					break;
			}
			DCMI_Start();//���¿�ʼ����
		} 

		if(track_flag)
		{
			if(MOSSE_Tracker.track())//center
			{
				//TrackFail_recovery
				current_local[0] = Rect.x;
				current_local[1] = Rect.y;
				
				MOSSE_Tracker.window_set(current_local[0] + Window_width*Cell_side*offset, current_local[1] );
				if(!MOSSE_Tracker.track())//right
					goto tracked;		
				
				MOSSE_Tracker.window_set(current_local[0], current_local[1] + Window_height*Cell_side*offset);
				if(!MOSSE_Tracker.track())//down
					goto tracked;
				
				MOSSE_Tracker.window_set(current_local[0] - Window_width*Cell_side/3, current_local[1] - Window_height*Cell_side/3);
				if(!MOSSE_Tracker.track())//up_left
					goto tracked;	
				
				MOSSE_Tracker.window_set(current_local[0], current_local[1] - Window_height*Cell_side*offset);
				if(!MOSSE_Tracker.track())//up
					goto tracked;
							
				MOSSE_Tracker.window_set(current_local[0] - Window_width*Cell_side*offset, current_local[1] );
				if(!MOSSE_Tracker.track())//left
					goto tracked;
										
				MOSSE_Tracker.window_set(current_local[0] + Window_width*Cell_side/3, current_local[1] + Window_height*Cell_side/3 );
				if(!MOSSE_Tracker.track())//buttom_right
					goto tracked;	
				
				MOSSE_Tracker.window_set(current_local[0] - Window_width*Cell_side/3, current_local[1] + Window_height*Cell_side/3 );
				if(!MOSSE_Tracker.track())//buttom_left
					goto tracked;	
				
				MOSSE_Tracker.window_set(current_local[0] + Window_width*Cell_side/3, current_local[1] - Window_height*Cell_side/3);
				if(!MOSSE_Tracker.track())//up_right
					goto tracked;

				POINT_COLOR = RED; //change window color				
				Rect.x = current_local[0] ;
				Rect.y = current_local[1] ;								
				DCMI_Pause();
				LCD_ShowString(Rect.x + Rect.width_x*Cell_side/2 - 45, Rect.y + Rect.width_y*Cell_side/2 - font_size - 6,420,font_size,font_size,"Tracking");
				LCD_ShowString(Rect.x + Rect.width_x*Cell_side/2 - 45, Rect.y + Rect.width_y*Cell_side/2						 + 6,420,font_size,font_size,"Failure");
				delay_ms(100);
				goto tracked;
//			DCMI_Restart();
				
			} else {
				POINT_COLOR=GREEN; //change window color
				goto tracked;
			}
		}
		tracked: 
		DCMI_Pause();
		LCD_DrawRectangle(Rect.x, Rect.y, Rect.x+Rect.width_x*Cell_side, Rect.y+Rect.width_y*Cell_side);
		LCD_DrawCross(Rect.x + Failure_Detection_Window_side*Cell_side, Rect.y + Failure_Detection_Window_side*Cell_side, 10);	//top_left
		LCD_DrawCross(Rect.x+Rect.width_x*Cell_side - Failure_Detection_Window_side*Cell_side, Rect.y + Failure_Detection_Window_side*Cell_side, 10);	//top_right
		LCD_DrawCross(Rect.x + Failure_Detection_Window_side*Cell_side, Rect.y+Rect.width_y*Cell_side - Failure_Detection_Window_side*Cell_side, 10);	//buttom_left
		LCD_DrawCross(Rect.x+Rect.width_x*Cell_side - Failure_Detection_Window_side*Cell_side, Rect.y+Rect.width_y*Cell_side - Failure_Detection_Window_side*Cell_side, 10);	//buttom_right
		DCMI_Restart();
	}    
}












