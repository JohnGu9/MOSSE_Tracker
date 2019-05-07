#include "myTrack.h"
#include "lcd.h"				//For getting pixel value from LCD 
#include "dcmi.h"				//Control image transmission
#include "malloc.h" 		//Allocate memory
#include "arm_math.h" 	//Access stm32f4 dsp
	
float* mark = NULL;
float* G = NULL;
float* Win = NULL;
u16*   Full_image = NULL;
float* fi = NULL;
float* Fi = NULL;
float* Fi_conj = NULL;
float* Ai = NULL;
float* Bi = NULL;
float* Hi = NULL;
float* gi = NULL;
float* cache = NULL;
rectangle Rect = {0,0,0,0,0};

_tracker MOSSE_Tracker = {
	myTrack_General_init,	//init
	myTrack_Rectreset,		//window_set
	myTrack,							//track
	myTrack_training			//train
};

void (*gen_win)(float* pSrc, const rectangle rect) = gen_hamming_win;	// choose window function
void (*preprocess)(float* pSrc, float* pDst, const rectangle rect) = rgbpreprocess;	//choose preprocess function

static arm_cfft_radix2_instance_f32 fft_xconf;
static arm_cfft_radix2_instance_f32 fft_yconf;
static arm_cfft_radix2_instance_f32 ifft_xconf; 
static arm_cfft_radix2_instance_f32 ifft_yconf;

volatile u32 jpeg_data_len=0; 
void jpeg_data_process(void){} 

//tip: Don't use STL log function for high compute consumption
static __INLINE float mFast_Log2(float val) 
{
    static union { float val; int32_t x; } u; 	//only for Single thread
		static float log_2;
		u.val = val;
    log_2 = (float)(((u.x >> 23) & 255) - 128);   //extract exponent           
    u.x   &= ~(255 << 23);
    u.x   |= 127 << 23;		//overwrite exponent
    return log_2 + ((-0.3358287811f) * u.val + 2.0f) * u.val  -0.65871759316667f;
} 
static void myTrack_General_init(u16 x, u16 y, u16 width_x, u16 width_y, u16 cell_side)
{
	myTrack_Rectset(x, y, width_x, width_y, cell_side);
	myTrack_init();
}
static void myTrack_Rectset(u16 x, u16 y, u16 width_x, u16 width_y, u16 cell_side)
{
	Rect.x = x;
	Rect.y = y;
	Rect.width_x = width_x;
	Rect.width_y = width_y;
	Rect.cell_side = cell_side;
}
static __INLINE void myTrack_Rectreset(int x, int y)
{
	//Boundary protection
	if(x < 0) Rect.x = 0;
	else if(x > lcddev.width - Rect.width_x*Cell_side - 1) Rect.x = lcddev.width - Rect.width_x*Cell_side - 1;
	else Rect.x = x;
	
	//Boundary protection
	if(y < 0) Rect.y = 0;
	else if(y > lcddev.height - Rect.width_y*Cell_side - 1) Rect.y = lcddev.height - Rect.width_y*Cell_side - 1;
	else Rect.y = y;
}
static void myTrack_init()
{
	arm_cfft_radix2_init_f32(&fft_xconf, Rect.width_x,0,1);
	arm_cfft_radix2_init_f32(&fft_yconf, Rect.width_y,0,1);

	arm_cfft_radix2_init_f32(&ifft_xconf, Rect.width_x,1,1);
	arm_cfft_radix2_init_f32(&ifft_yconf, Rect.width_y,1,1);
	
	//Allocate memory
	G = (float*)mymalloc(SRAMEX, 2*Rect.width_x*Rect.width_y*sizeof(float));
	Win = (float*)mymalloc(SRAMIN, Rect.width_x*Rect.width_y*sizeof(float));
	//Full_image = (u16*)mymalloc(SRAMEX, 4*Rect.width_x*Rect.width_y*sizeof(u16));
	fi = (float*)mymalloc(SRAMCCM, Rect.width_x*Rect.width_y*sizeof(float));
	Fi = (float*)mymalloc(SRAMIN, 2*Rect.width_x*Rect.width_y*sizeof(float));
	Fi_conj = (float*)mymalloc(SRAMEX, 2*Rect.width_x*Rect.width_y*sizeof(float));
	Ai = (float*)mymalloc(SRAMEX, 2*Rect.width_x*Rect.width_y*sizeof(float));
	Bi = (float*)mymalloc(SRAMEX, 2*Rect.width_x*Rect.width_y*sizeof(float));	
	Hi = (float*)mymalloc(SRAMEX, 2*Rect.width_x*Rect.width_y*sizeof(float));
	gi = (float*)mymalloc(SRAMIN, 2*Rect.width_x*Rect.width_y*sizeof(float));
	cache= (float*)mymalloc(SRAMCCM, 2*Rect.width_x*Rect.width_y*sizeof(float));
	
	while(G==NULL || Win==NULL || Fi==NULL || fi==NULL 
		|| Fi_conj==NULL || Ai==NULL || Bi==NULL 
		|| Hi==NULL || gi==NULL || cache==NULL)
	{
		LCD_ShowString(30,50,300,16,16,"Memory Initialization Error");
	}//check memory	

	gen_win(Win, Rect);
	gen_Gauss(G, Rect);
}

//return 0	track success
//return 1	track fail
static __INLINE int myTrack()
{
	static u16 max_x;
	static u16 max_y;
	static int fd_flag;

#ifndef	ONLINE_TRAIN
	//high prefromance without train
	//catch image from camera
	DCMI_Pause();		
	get_img(fi, &Rect);	//get image			
	DCMI_Restart();	
	
	preprocess(fi, Fi, Rect);			
	fft2(Fi, &Rect);	
	Mat_mult_Mat(Hi, Fi, gi, Rect);
	ifft2(gi, &Rect);
	find_max_local(gi, Rect, &max_x, &max_y);
	fd_flag = Failure_Detection(gi, cache, &max_x, &max_y);//2 well, 1 work, 0 loss
	if(!fd_flag)
	{
		return 1;
	} else {
		update_rect(&Rect, &max_x, &max_y);//if no Failure_detection, update the Rect
		return 0;
	}
#else
	//low prefromance with train
	//catch image from camera
	DCMI_Pause();		
	get_img(fi, &Rect);	//get image			
			
	preprocess(fi, Fi, Rect);			
	fft2(Fi, &Rect);		//fft fi	
	Mat_mult_Mat(Hi, Fi, gi, Rect);
	ifft2(gi, &Rect);
	find_max_local(gi, Rect, &max_x, &max_y);
	fd_flag = Failure_Detection(gi, cache, &max_x, &max_y);//2 well 1 work 0 loss
	if(fd_flag == 0)
	{
		DCMI_Restart();
		return 1;
	} else if(fd_flag == 1){
		update_rect(&Rect, &max_x, &max_y);	//if no Failure_detection, update the Rect
		myTrack_training(Rect);  				 		//track not well, with train
		DCMI_Restart();
		return 0;
	} else if(fd_flag == 2){
		update_rect(&Rect, &max_x, &max_y);	//if no Failure_detection, update the Rect
		//myTrack_training(Rect);  					//track well, without train
		DCMI_Restart();
		return 0;
	}
	return 1;//unknow state
#endif
	
}
static __INLINE void myTrack_training(rectangle rect)
{
	static u8 Hi_available = 0;
	get_img(fi, &rect);	
	//LCD_DrawRectangle(rect.x,rect.y,rect.x+rect.width_x*Cell_side,rect.y+rect.width_y*Cell_side);
	preprocess(fi, Fi, rect);
	fft2(Fi, &rect);		
	conj_Fi(Fi, Fi_conj, rect);
	if(Hi_available)
	{
		update_Ai(Ai, G,  Fi_conj, rect);
		update_Bi(Bi, Fi, Fi_conj, rect);
		update_Hi(Hi, Ai, Bi, rect);	
	} else {
		gen_Ai(Ai, G, Fi_conj, rect);
		gen_Bi(Bi, Fi, Fi_conj, rect);
		update_Hi(Hi, Ai, Bi, rect);	
		Hi_available = 1;
	}
}

//get a square image with a side length of "side" from LCD interface
//one by one ROW, for example: array format [frist row, second row, .......]
//a cell square with a side length of "merga_side", all pixel of this square will be Blended into one pixel 
//NOT overlap
//!stop DCMI! before get_pic
static __INLINE void get_img(float* pSrc, const rectangle* const rect)	
{
	static u16 row, col;
	for(row = 0;row < rect->width_y * rect->cell_side; row += rect->cell_side)
		for(col = 0;col < rect->width_x * rect->cell_side; col += rect->cell_side)
			*pSrc++ = *(get_merge_avg(rect->x+col, rect->y+row, rect->cell_side)); //+1 for log function in preprocess
}
void get_full_pic(u16* pSrc, const rectangle* const rect)	//
{
	static int row, col;
	for(row = -rect->width_y*rect->cell_side/2; row < 3*rect->width_y*rect->cell_side/2;row += rect->cell_side)
		for(col = -rect->width_x*rect->cell_side/2; col < 3*rect->width_x*rect->cell_side/2;col += rect->cell_side)
			if(rect->x+col<0)*pSrc++ = 1;
			else if(rect->y+row<0)*pSrc++ = 1;
			else *pSrc++ = *(get_merge_avg((rect->x+col), (rect->y+row), rect->cell_side));
}
//mix	merga_side*merga_side pixels into one pixel
//only get one color channel(recommend using B&W mode before use this function)
static __INLINE u16* get_merge_avg(const u16 x_topleft, const u16 y_topleft, const u16 cell_side)	
{
	static u16 pixel;
	static u16 row, col;
	pixel = 0x0001;//preprocess
	for(row = 0; row < cell_side; row++)
		for(col = 0; col < cell_side; col++)
			pixel += LCD_ReadPoint(x_topleft+col, y_topleft+row) & 0x001F;		//only extract blue channel

	return &pixel;
}

//2_dim fft
static __INLINE void fft2(float* pSrc, const rectangle* const rect)	
{
	static u16 i, j;
		
	for(i = 0; i < rect->width_y; i++) 
		arm_cfft_radix2_f32(&fft_xconf, pSrc+2*i*rect->width_x);

	for(i = 0; i < 2*rect->width_x; i += 2) 
	{
		for(j = 0; j < 2*rect->width_y; j += 2)
		{
			*(cache+j) = *(pSrc+j*rect->width_x+i);
			*(cache+j+1) = *(pSrc+j*rect->width_x+i+1);
		}	//read
		arm_cfft_radix2_f32(&fft_yconf, cache);
		for(j = 0; j < 2*rect->width_y; j += 2)
		{
			 *(pSrc+j*rect->width_x+i)=*(cache+j);
			 *(pSrc+j*rect->width_x+i+1)=*(cache+j+1);
		}	//write
	}	
}
//2_dim ifft
static __INLINE void ifft2(float* pSrc, const rectangle* const rect)	
{
	static u16 i, j;
		
	for(i = 0; i < rect->width_y; i++) 
		arm_cfft_radix2_f32(&ifft_xconf, pSrc+2*i*rect->width_x);

	for(i = 0; i < 2*rect->width_x; i += 2) 
	{
		for(j = 0; j < 2*rect->width_y; j += 2)
		{
			*(cache+j) = *(pSrc+j*rect->width_x+i);
			*(cache+j+1) = *(pSrc+j*rect->width_x+i+1);
		}	//read
		arm_cfft_radix2_f32(&ifft_yconf, cache);
		for(j = 0; j < 2*rect->width_y; j += 2)
		{
			 *(pSrc+j*rect->width_x+i)=*(cache+j);
			 *(pSrc+j*rect->width_x+i+1)=*(cache+j+1);
		}	//write
	}	
}


//Generation of 2_dim hanning window
void gen_hann_win(float* pSrc, const rectangle rect)	
{
	u16 i, j;
	for(i = 0; i < rect.width_y; i++)	//y
		for(j = 0; j < rect.width_x; j++)	//x
			*(pSrc+i*rect.width_x+j) = ((1.f/2.f - 1.f/2.f * arm_cos_f32(2*PI*i/(rect.width_y-1))))
																*((1.f/2.f - 1.f/2.f * arm_cos_f32(2*PI*j/(rect.width_x-1))));
}

void gen_hamming_win(float* pSrc, const rectangle rect)
{
	u16 i, j;
	for(i = 0; i < rect.width_y; i++)	//y
		for(j = 0; j < rect.width_x; j++)	//x
			*(pSrc+i*rect.width_x+j) = ((25.f/46.f - 21.f/46.f * arm_cos_f32(2*PI*i/(rect.width_y-1))))	
																*((25.f/46.f - 21.f/46.f * arm_cos_f32(2*PI*j/(rect.width_x-1))));
}
//Generation of 2_dim Gauss Fliter
void gen_Gauss(float* pSrc, const rectangle rect)	
{
	int i, j;
	int widthx, widthy;
	widthx = rect.width_x/2;	
	widthy = rect.width_y/2;

	for(i = 0; i < rect.width_y; i++)	//y
		for(j = 0; j < rect.width_x; j++)	//x
		{
			*(pSrc+2*i*rect.width_x+2*j) = exp(-(((i-widthy+1)*(i-widthy+1)+(j-widthx+1)*(j-widthx+1))/Sigma));	
			*(pSrc+2*i*rect.width_x+2*j+1) = 0.f;
		}
	fft2(pSrc, &rect);
}

inline static void rgbpreprocess(float* pSrc, float* pDst, const rectangle rect)
{
	static float mean, std;	
	//Mat_log
	static u16 i;
	for(i = 0; i < rect.width_x*rect.width_y; i++)
		*(pSrc+i) = mFast_Log2(*(pSrc+i)/*+1.f*/);	//get_img function have already +1 for every pixels

	arm_mean_f32(pSrc, rect.width_x*rect.width_y, &mean);
	arm_std_f32(pSrc, rect.width_x*rect.width_y, &std);
	std += 0.00001f;
	
	for(i = 0; i < rect.width_x*rect.width_y; i++)
	{
		*(pDst+2*i) = ((*(pSrc+i)-mean)/std)*(*(Win+i));	//make sure init for Win
		*(pDst+2*i+1) = 0.f;
	}	
}
inline void hogpreprocess(float* pSrc, float* pDst, const rectangle rect)// unfinished
{
	static u16 i, j;
	for(i = 0; i < rect.width_y; i++)
		for(j = 0; j < rect.width_x; j++)
		{
			*(pDst+2*(i*rect.width_x+j)) = *(pSrc+(i*rect.width_x+(j == rect.width_x - 1? j : j+1))) - *(pSrc+(i*rect.width_x+(j == 0? j : j-1)));
			*(pDst+2*(i*rect.width_x+j)+1) = *(pSrc+((i == rect.width_y - 1? i : i+1)*rect.width_x+j)) - *(pSrc+((i == 0? i : i-1)*rect.width_x+j));
		}	
}

inline static void conj_Fi(float* Fi, float* Fi_conj, rectangle rect)
{
	arm_cmplx_conj_f32(Fi, Fi_conj, rect.width_x*rect.width_y);
}	
inline static void gen_Ai(float* Ai, float* G, float* Fi_conj, rectangle rect)
{
	arm_cmplx_mult_cmplx_f32(G, Fi_conj, Ai, rect.width_x*rect.width_y);
}
inline static void gen_Bi(float* Bi, float* Fi, float* Fi_conj, rectangle rect)
{
	arm_cmplx_mult_cmplx_f32(Fi, Fi_conj, Bi, rect.width_x*rect.width_y);
}

inline static void update_Ai(float* Ai, float* G, float* Fi_conj, rectangle rect)
{
	u16 i;
	arm_cmplx_mult_cmplx_f32(G, Fi_conj, cache, rect.width_x*rect.width_y);
	for(i = 0; i < 2*rect.width_x*rect.width_y; i++)
	{
		(*(Ai+i)) = (*(Ai+i)) * (1-Update_rate) + (*(cache+i)) * Update_rate;
	}
}
inline static void update_Bi(float* Bi, float* Fi, float* Fi_conj, rectangle rect)
{
	u16 i;
	arm_cmplx_mult_cmplx_f32(Fi, Fi_conj, cache, rect.width_x*rect.width_y);
	for(i = 0; i < 2*rect.width_x*rect.width_y; i++)
	{
		(*(Bi+i)) = (*(Bi+i)) * (1-Update_rate) + (*(cache+i)) * Update_rate;
	}
}
inline static void update_Hi(float* Hi, float* Ai, float* Bi, rectangle rect)
{
	static u16 i;
	for(i = 0; i < rect.width_x*rect.width_y; i++)
	{
		*(Hi+2*i) = (*(Ai+2*i)) / (*(Bi+2*i));
		*(Hi+2*i+1) = (*(Ai+2*i+1)) / (*(Bi+2*i));
	}
}
//Mat Point multiplication!
inline static void Mat_mult_Mat(float* Hi, float* Fi, float* gi, const rectangle rect)	
{
		arm_cmplx_mult_cmplx_f32(Hi, Fi, gi, rect.width_x*rect.width_y);
}

inline static float find_max_local(float* gi, const rectangle rect, u16* max_x, u16* max_y)
{
	static u16 i, j;
	static float max;
	max = *gi;
	for(i = 0; i < rect.width_x*rect.width_y; i++)
	{
		if(*(gi+2*i) >= max)
		{
			max = *(gi+2*i);
			j = i;
		}		
	}
	*max_x = j%rect.width_x+1;	//Compensation offset
	*max_y = j/rect.width_x+1;	//Compensation offset `
	return max;
}
inline static void extract_gi(float* gi, float* cache, const rectangle rect)	
{
	static u16 i;
	//extract real val of Mat gi
	for(i = 0; i <rect.width_x*rect.width_y; i++)
		*(cache+i) = *(gi+2*i);
}

inline static float PSR(float* cache, const rectangle rect, u16* max_x, u16* max_y)
{
	static int i, j;
	float* clc;
	static float mean, std, max;
	static int pixels;
	max = *(cache + *max_x + *max_y * rect.width_x);
	clc = cache + *max_x + *max_y * rect.width_x;
	pixels = rect.width_y*rect.width_x;	//total pixels of image
	
	for(i = - Failure_Detection_Window_side/2; i < Failure_Detection_Window_side/2 + 1; i++)
		for(j = - Failure_Detection_Window_side/2; j < Failure_Detection_Window_side/2 + 1; j++)
			*(clc+j+i*rect.width_x 
				+ (*max_x < -j? rect.width_x : 0) - (*max_x + j >= rect.width_x? rect.width_x : 0)		//Circulant Structure of bounding 
				+ (*max_y < -i? pixels : 0)			  - (*max_y + i >= rect.width_y? pixels : 0))				//Circulant Structure of bounding 
				= 0.f; 
	
	arm_mean_f32(cache, rect.width_x*rect.width_y, &mean);
	mean = mean * rect.width_x*rect.width_y / (rect.width_x*rect.width_y - Failure_Detection_Window_side*Failure_Detection_Window_side);
	
	for(i = - Failure_Detection_Window_side/2; i < Failure_Detection_Window_side/2 + 1; i++)
		for(j = - Failure_Detection_Window_side/2; j < Failure_Detection_Window_side/2 + 1; j++)
			*(clc+j+i*rect.width_x
				+ (*max_x < -j? rect.width_x : 0) - (*max_x + j >= rect.width_x? rect.width_x : 0)		//Circulant Structure of bounding 
				+ (*max_y < -i? pixels : 0)			  - (*max_y + i >= rect.width_y? pixels : 0))				//Circulant Structure of bounding 
				= mean;
	
	arm_std_f32(cache, rect.width_x*rect.width_y, &std);
	return (max - mean)/std;		

}
//return 2 Well track 
//return 1 no Failure_detected
//return 0 Failure_detected
inline static int Failure_Detection(float* gi, float* cache, u16* max_x, u16* max_y)
{
	static float psr;
	extract_gi(gi, cache, Rect);
	psr = PSR(cache, Rect, max_x, max_y);
	if(psr >= PSR_Train_Threshold)
		return 2;
	else if(psr >= PSR_Threshold)
		return 1;

	return 0;
}
inline static void update_rect(rectangle* const rect, u16* max_x, u16* max_y)
{
	//update point with checking validity of this point 
	if(rect->x + (*max_x)*rect->cell_side < rect->width_x*rect->cell_side/2)
	{
		rect->x = 0;	
	} else if(rect->x + (*max_x)*rect->cell_side + rect->width_x*rect->cell_side/2 >= lcddev.width-1) {
		rect->x = lcddev.width - rect->width_x*rect->cell_side-1;
	} else {
		rect->x = rect->x + (*max_x)*rect->cell_side - rect->width_x*rect->cell_side/2;
	}

	if(rect->y+(*max_y)*rect->cell_side < rect->width_y*rect->cell_side/2)
	{
		rect->y = 0;		
	} else if (rect->y + (*max_y)*rect->cell_side + rect->width_y*rect->cell_side/2 >= lcddev.height-1) {
		rect->y = lcddev.height - rect->width_y*rect->cell_side-1;
	}else {
		rect->y = rect->y + (*max_y)*rect->cell_side - rect->width_y*rect->cell_side/2;
	}
}





