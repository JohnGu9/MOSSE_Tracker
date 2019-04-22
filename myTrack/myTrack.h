#ifndef __MYTRACK_H
#define __MYTRACK_H
#include "sys.h"
#include "math.h" 
#include "arm_math.h" 

#define ONLINE_TRAIN	//online train switch

#define Sigma 8.f	// Sigma = 2*sigma^2 
#define Window_width 64
#define Window_height 64
#define Cell_side 3
#define Update_rate 0.125f
#define Failure_Detection_Window_side 11
#define PSR_Threshold 7.6f
#define PSR_Train_Threshold 24.f
#define font_size 24

extern float* mark;
extern float* G;
extern float* Win;
extern u16*   Full_image;
extern float* fi;
extern float* Fi;
extern float* Fi_conj;
extern float* Ai;
extern float* Bi;
extern float* Hi;
extern float* gi;
extern float* cache;

typedef struct {
	u16 x;	//top_left corner
	u16 y;	
	u16 width_x;
	u16 width_y;
	u16 cell_side;
}rectangle;
extern rectangle Rect;	//Tracker Window


typedef struct {
 void (*init)(u16 x, u16 y, u16 width_x, u16 width_y, u16 cell_side);
 void (*window_set)(int x, int y);
 int  (*track)(void);
 void (*train)(rectangle rect);
}_mossetrack;
extern _mossetrack MOSSE_Tracker;


//Tracker Interface
static void myTrack_General_init(u16 x, u16 y, u16 width_x, u16 width_y, u16 cell_side);
static void myTrack_Rectset(u16 x, u16 y, u16 width_x, u16 width_y, u16 cell_side);
static void myTrack_Rectreset(int x, int y);
static void myTrack_init(void);
static int  myTrack(void);
static void myTrack_training(rectangle rect);

//Internal Implementation
static u16* get_merge_avg(const u16 x_topleft, const u16 y_topleft, const u16 cell_side);
static void get_img(float* output, const rectangle* const rect);
void get_full_pic(u16* full_image, const rectangle* const rect);

static void fft2(float* pSrc, const rectangle* const rect);			//This "fftLen" is the side length of square, just as a parameter of fft function mentioned above
static void ifft2(float* pSrc, const rectangle* const rect);		//This "ifftLen" is the side length of square, just as a parameter of ifft function mentioned above

//static void (*gen_win)(float* pSrc, const rectangle rect);
//static void (*preprocess)(float* pSrc, float* pDst, const rectangle rect);
void gen_hamming_win(float* pSrc, const rectangle rect);
void gen_hann_win(float* pSrc, const rectangle rect);
void gen_Gauss(float* pSrc, const rectangle rect);
void static rgbpreprocess(float* pSrc, float* pDst, const rectangle rect);
void hogpreprocess(float* pSrc, float* pDst, const rectangle rect);
static void conj_Fi(float* Fi, float* Fi_conj, rectangle rect);

static void gen_Ai(float* Ai, float* G, float* Fi_conj, rectangle rect);
static void gen_Bi(float* Bi, float* Fi, float* Fi_conj, rectangle rect);
static void update_Ai(float* Ai, float* G, float* Fi_conj, rectangle rect);
static void update_Bi(float* Bi, float* Fi, float* Fi_conj, rectangle rect);
static void update_Hi(float* Hi, float* Ai, float* Bi, rectangle rect);

static void Mat_mult_Mat(float* gi, float* Hi, float* Fi, const rectangle rect);
static float find_max_local(float* gi, const rectangle rect, u16* max_x, u16* max_y);
static void extract_gi(float* gi, float* cache, const rectangle rect);
static float PSR(float* cache, const rectangle rect, u16* max_x, u16* max_y);
static int Failure_Detection(float* gi, float* cache, u16* max_x, u16* max_y);
static void update_rect(rectangle* const rect, u16* max_x, u16* max_y);

static void rgb565(void);
#endif
