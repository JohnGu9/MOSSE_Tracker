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

//#define Performance_Track

#define offset 4/10
u8 timeout;
float time; 


int main(void)
{
	FSMC_SRAM_Init();				//初始化外部SRAM  	
	my_mem_init(SRAMIN);		//初始化内部内存池
	my_mem_init(SRAMEX);		//初始化外部内存池
	my_mem_init(SRAMCCM);		//初始化CCM内存池	
//	Stm32_Clock_Init(378,8,2,7);//设置时钟,182Mhz 
	Stm32_Clock_Init(336,8,2,7);//设置时钟,168Mhz 
	delay_init(189);			//延时初始化  
	uart_init(84,115200);		//初始化串口波特率为115200 
	LED_Init();					//初始化LED 
 	LCD_Init();					//LCD初始化  
 	KEY_Init();					//按键初始化
#ifdef performance
	TIM3_Int_Init(65535,8400-1);//10Khz计数频率,最大计时6.5秒超出
#endif	
 	POINT_COLOR=RED;//设置字体为红色 
	
	MOSSE_Tracker.init(	lcddev.width/2 - Window_width*Cell_side/2, 
											lcddev.height/2 - Window_height*Cell_side/2,
											Window_width,
											Window_height,
											Cell_side);	//Tracker Initialization

	LCD_ShowString(30,50,600,font_size,font_size,"Explorer STM32F4");	
	LCD_ShowString(30,50+30,600,font_size,font_size,"MOSSE_Track for Embedded Systems");	
	LCD_ShowString(30,50+30*2,600,font_size,font_size,"johngumaster@outlook.com");
	LCD_ShowString(30,50+30*3,600,font_size,font_size,"2019/3/29");
	
	while(OV2640_Init())//初始化OV2640
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
inline void myLCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p)
{         
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        LCD_ShowChar(x,y,*p,size,0);
        x+=size/2;
        p++;
    }  
}
inline static void rgb565(void)
{ 
	static u8 key;
	static u16 default_local[2];
	static int current_local[2];
#ifdef Performance_Track
	static unsigned char buf[20];
#endif
	
	static u8 track_flag;
	static u8 Hi_available;
	track_flag = 0;
	Hi_available = 0;

	default_local[0] = lcddev.width/2 - Window_width*Cell_side/2;
	default_local[1] = lcddev.height/2 - Window_height*Cell_side/2;
	
	LCD_Clear(WHITE);
  POINT_COLOR=YELLOW; //change window color 
	OV2640_RGB565_Mode();	//RGB565模式
	DCMI_Init();			//DCMI配置
	DCMI_DMA_Init((u32)&LCD->LCD_RAM,1,1,0);//DCMI DMA配置 
#ifndef Horizon
//vertical
	OV2640_ImageWin_Set((1600-1200*lcddev.width/lcddev.height)/2,0,1200*lcddev.width/lcddev.height,1200);//真实尺寸
#else
//horizon 	
	OV2640_ImageWin_Set(0,(1200-1600*lcddev.width/lcddev.height)/2,1600,1600*lcddev.width/lcddev.height);//真实尺寸
#endif
	OV2640_OutSize_Set(lcddev.width,lcddev.height);
	OV2640_Contrast(3);//Increase contrast a bit 
	DCMI_Start(); 		//启动传输
	delay_ms(200); 
	while(1)
	{ 
		key=KEY_Scan(0); 
		if(key)
		{ 
			POINT_COLOR = YELLOW; //change window color	
			DCMI_Stop(); //停止显示
			switch(key)
			{				    
				case KEY0_PRES:	//对比度设置
					
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
				case KEY2_PRES:	//特效设置				 

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
			DCMI_Start();//重新开始传输
		} 
#ifdef Performance_Track
//timer setup
		TIM3->CNT=0;//重设TIM3定时器的计数器值
		timeout=0;
//timer setup
	#endif
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
#ifdef Performance_Track		
//timer statistics time
		time=TIM3->CNT+(u32)timeout*65536;
		sprintf((char*)buf, "Time:%f\r\n",time);//打印帧率
		myLCD_ShowString(30,50,600,font_size,font_size,buf);
		delay_ms(500);		
//timer statistics time		
#endif
		LCD_DrawRectangle(Rect.x, Rect.y, Rect.x+Rect.width_x*Cell_side, Rect.y+Rect.width_y*Cell_side);
		LCD_DrawCross(Rect.x + Failure_Detection_Window_side*Cell_side, Rect.y + Failure_Detection_Window_side*Cell_side, 10);	//top_left
		LCD_DrawCross(Rect.x+Rect.width_x*Cell_side - Failure_Detection_Window_side*Cell_side, Rect.y + Failure_Detection_Window_side*Cell_side, 10);	//top_right
		LCD_DrawCross(Rect.x + Failure_Detection_Window_side*Cell_side, Rect.y+Rect.width_y*Cell_side - Failure_Detection_Window_side*Cell_side, 10);	//buttom_left
		LCD_DrawCross(Rect.x+Rect.width_x*Cell_side - Failure_Detection_Window_side*Cell_side, Rect.y+Rect.width_y*Cell_side - Failure_Detection_Window_side*Cell_side, 10);	//buttom_right
		DCMI_Restart();
	}    
}












