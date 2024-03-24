#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <fstream>
#include <windows.h>
#include <string>
#include <vector>
#include <iostream>
#include <direct.h>
#include <stdlib.h>
#include <chrono>

#define _CRT_SECURE_NO_WARNINGS

using namespace std; //C++ 語法用
using namespace cv;

#define SQ(x) ((x)*(x))

int W, H; // image width, height
int nChannels=3, nChannels_guide=3;
double* BLFKernelI; // Kernel LUT

// Main functions
void prepareBLFKernel(double sigma);
void FGS_simple(double*** image, double*** joint_image, double sigma, double lambda, int solver_iteration, double solver_attenuation);
void solve_tridiagonal_in_place_destructive(double x[], const size_t N, const double a[], const double b[], double c[]);

// Memory management
double*** memAllocDouble3(int n, int r, int c);
double** memAllocDouble2(int r, int c);
void memFreeDouble3(double*** p);
void memFreeDouble2(double** p);


// Build LUT for bilateral kernel weight
void prepareBLFKernel(double sigma)
{
	const int MaxSizeOfFilterI = 195075;
	BLFKernelI = (double*)malloc(sizeof(double) * MaxSizeOfFilterI);

	for (int m = 0; m < MaxSizeOfFilterI; m++)
		BLFKernelI[m] = exp(-sqrt((double)m) / (sigma)); // Kernel LUT
}

// mex function call:
// x = mexFGS(input_image, guidance_image = NULL, sigma, lambda, fgs_iteration = 3, fgs_attenuation = 4);
void mexFunction(Mat img)
{
	
	// image resolution
	W = img.cols;
	H = img.rows;
	
	nChannels = 3;
	

	// FGS parameters
	double sigma = 0.01;//0.01 my set
	double lambda = pow(20, 2);
	int solver_iteration = 3;
	double solver_attenuation = 4.0;



	// Image buffer preperation
	double*** image_filtered = memAllocDouble3(H, W, nChannels);
	//double* ptr_image = (double*)mxGetPr(img);
	double* ptr_image_array = image_filtered[0][0];
	for (int y = 0; y < H; y++) {
		for (int x = 0; x < W; x++) {
			for (int c = 0; c < nChannels; c++) {
				//image_filtered[y][x][c] = (double)ptr_image[x*H + y + c*(H*W)];
				double val = img.at<Vec3b>(y, x)[c];
				ptr_image_array[y * W * nChannels + x * nChannels + c] = val;
			}
		}
	}

	double*** image_guidance = NULL;
	
	image_guidance = memAllocDouble3(H, W, nChannels_guide);

	//double* ptr_guidance = (double*)mxGetPr(imgGuide);
	double* ptr_guidance_array = image_guidance[0][0];
	for (int y = 0; y < H; y++) 
		for (int x = 0; x < W; x++) 
			for (int c = 0; c < nChannels_guide; c++) {
				double val = img.at<Vec3b>(y, x)[c];
				ptr_guidance_array[y * W * nChannels_guide + x * nChannels_guide + c] = val;
			}
				


	// run FGS
	sigma *= 255.0;

	clock_t m_begin = clock(); // time measurement;
	FGS_simple(image_filtered, image_guidance, sigma, lambda, solver_iteration, solver_attenuation);
	
	// output
	Mat integral = Mat::zeros(H, W, CV_8UC3);
	
	for (int y = 0; y < H; y++) 
		for (int x = 0; x < W; x++) 
			for (int c = 0; c < nChannels; c++) 
				integral.at<Vec3b>(y, x)[c] = image_filtered[y][x][c];


	imwrite("integral_.jpg", integral);
	memFreeDouble3(image_filtered);
	if (image_guidance) memFreeDouble3(image_guidance);
}


// mex function call:
// x = mexFGS(input_image, guidance_image = NULL, sigma, lambda, fgs_iteration = 3, fgs_attenuation = 4);
void mexFunction_2(Mat img,Mat disp)
{

	// image resolution
	W = img.cols;
	H = img.rows;

	nChannels = 3;


	// FGS parameters
	double sigma = 0.06;//0.06 my set
	double lambda = pow(30, 2);
	int solver_iteration = 3;
	double solver_attenuation = 4.0;



	// Image buffer preperation
	double*** image_filtered = memAllocDouble3(H, W, nChannels);
	//double* ptr_image = (double*)mxGetPr(img);
	double* ptr_image_array = image_filtered[0][0];
	for (int y = 0; y < H; y++) {
		for (int x = 0; x < W; x++) {
			for (int c = 0; c < nChannels; c++) {
				//image_filtered[y][x][c] = (double)ptr_image[x*H + y + c*(H*W)];
				double val = img.at<Vec3b>(y, x)[c];
				ptr_image_array[y * W * nChannels + x * nChannels + c] = val;
			}
		}
	}

	double*** image_guidance = NULL;

	image_guidance = memAllocDouble3(H, W, nChannels_guide);

	//double* ptr_guidance = (double*)mxGetPr(imgGuide);
	double* ptr_guidance_array = image_guidance[0][0];
	for (int y = 0; y < H; y++)
		for (int x = 0; x < W; x++)
			for (int c = 0; c < nChannels_guide; c++) {
				double val = img.at<Vec3b>(y, x)[c];
				ptr_guidance_array[y * W * nChannels_guide + x * nChannels_guide + c] = val;
			}



	// run FGS
	sigma *= 255.0;

	clock_t m_begin = clock(); // time measurement;
	FGS_simple(image_filtered, image_guidance, sigma, lambda, solver_iteration, solver_attenuation);

	// output
	Mat integral = Mat::zeros(H, W, CV_8UC3);

	for (int y = 0; y < H; y++)
		for (int x = 0; x < W; x++)
			for (int c = 0; c < nChannels; c++)
				integral.at<Vec3b>(y, x)[c] = image_filtered[y][x][c];


	imwrite("integral_disp_0102.jpg", integral);
	memFreeDouble3(image_filtered);
	if (image_guidance) memFreeDouble3(image_guidance);
}

void FGS_simple(double*** image, double*** joint_image, double sigma, double lambda, int solver_iteration, double solver_attenuation)
{
	int color_diff;

	int width = W;
	int height = H;

	if (joint_image == NULL) joint_image = image;

	double* a_vec = (double*)malloc(sizeof(double) * width);
	double* b_vec = (double*)malloc(sizeof(double) * width);
	double* c_vec = (double*)malloc(sizeof(double) * width);
	double* x_vec = (double*)malloc(sizeof(double) * width);
	double* c_ori_vec = (double*)malloc(sizeof(double) * width);

	double* a2_vec = (double*)malloc(sizeof(double) * height);
	double* b2_vec = (double*)malloc(sizeof(double) * height);
	double* c2_vec = (double*)malloc(sizeof(double) * height);
	double* x2_vec = (double*)malloc(sizeof(double) * height);
	double* c2_ori_vec = (double*)malloc(sizeof(double) * height);

	prepareBLFKernel(sigma);

	//Variation of lambda (NEW)
	double lambda_in = 1.5 * lambda * pow(4.0, solver_iteration - 1) / (pow(4.0, solver_iteration) - 1.0);
	for (int iter = 0; iter < solver_iteration; iter++)
	{
		//for each row
		for (int i = 0; i < height; i++)
		{
			memset(a_vec, 0, sizeof(double) * width);
			memset(b_vec, 0, sizeof(double) * width);
			memset(c_vec, 0, sizeof(double) * width);
			memset(c_ori_vec, 0, sizeof(double) * width);
			memset(x_vec, 0, sizeof(double) * width);
			for (int j = 1; j < width; j++)
			{
				int color_diff = 0;
				// compute bilateral weight for all channels
				for (int c = 0; c < nChannels_guide; c++)
					color_diff += SQ(joint_image[i][j][c] - joint_image[i][j - 1][c]);

				a_vec[j] = -lambda_in * BLFKernelI[color_diff];		//WLS
			}
			for (int j = 0; j < width - 1; j++)	c_ori_vec[j] = a_vec[j + 1];
			for (int j = 0; j < width; j++)	b_vec[j] = 1.f - a_vec[j] - c_ori_vec[j];		//WLS

			for (int c = 0; c < nChannels; c++)
			{
				memcpy(c_vec, c_ori_vec, sizeof(double) * width);
				for (int j = 0; j < width; j++)	x_vec[j] = image[i][j][c];
				solve_tridiagonal_in_place_destructive(x_vec, width, a_vec, b_vec, c_vec);
				for (int j = 0; j < width; j++)	image[i][j][c] = x_vec[j];
			}
		}

		//for each column
		for (int j = 0; j < width; j++)
		{
			memset(a2_vec, 0, sizeof(double) * height);
			memset(b2_vec, 0, sizeof(double) * height);
			memset(c2_vec, 0, sizeof(double) * height);
			memset(c2_ori_vec, 0, sizeof(double) * height);
			memset(x2_vec, 0, sizeof(double) * height);
			for (int i = 1; i < height; i++)
			{
				int color_diff = 0;
				// compute bilateral weight for all channels
				for (int c = 0; c < nChannels_guide; c++)
					color_diff += SQ(joint_image[i][j][c] - joint_image[i - 1][j][c]);

				a2_vec[i] = -lambda_in * BLFKernelI[color_diff];		//WLS
			}
			for (int i = 0; i < height - 1; i++)
				c2_ori_vec[i] = a2_vec[i + 1];
			for (int i = 0; i < height; i++)
				b2_vec[i] = 1.f - a2_vec[i] - c2_ori_vec[i];		//WLS

			for (int c = 0; c < nChannels; c++)
			{
				memcpy(c2_vec, c2_ori_vec, sizeof(double) * height);
				for (int i = 0; i < height; i++)	x2_vec[i] = image[i][j][c];
				solve_tridiagonal_in_place_destructive(x2_vec, height, a2_vec, b2_vec, c2_vec);
				for (int i = 0; i < height; i++)	image[i][j][c] = x2_vec[i];
			}
		}

		//Variation of lambda (NEW)
		lambda_in /= solver_attenuation;
	}	//iter	

	free(a_vec);
	free(b_vec);
	free(c_vec);
	free(x_vec);
	free(c_ori_vec);

	free(a2_vec);
	free(b2_vec);
	free(c2_vec);
	free(x2_vec);
	free(c2_ori_vec);
}

void solve_tridiagonal_in_place_destructive(double x[], const size_t N, const double a[], const double b[], double c[])
{
	int n;

	c[0] = c[0] / b[0];
	x[0] = x[0] / b[0];

	// loop from 1 to N - 1 inclusive 
	for (n = 1; n < N; n++) {
		double m = 1.0f / (b[n] - a[n] * c[n - 1]);
		c[n] = c[n] * m;
		x[n] = (x[n] - a[n] * x[n - 1]) * m;
	}

	// loop from N - 2 to 0 inclusive 
	for (n = N - 2; n >= 0; n--)
		x[n] = x[n] - c[n] * x[n + 1];
}

double*** memAllocDouble3(int n, int r, int c)
{
	int padding = 10;
	double* a, ** p, *** pp;
	int rc = r * c;
	int i, j;
	a = (double*)malloc(sizeof(double) * (n * rc + padding));
	//if (a == NULL) { mexErrMsgTxt("memAllocDouble: Memory is too huge.\n"); }
	p = (double**)malloc(sizeof(double*) * n * r);
	pp = (double***)malloc(sizeof(double**) * n);
	for (i = 0; i < n; i++)
		for (j = 0; j < r; j++)
			p[i * r + j] = &a[i * rc + j * c];
	for (i = 0; i < n; i++)
		pp[i] = &p[i * r];
	return(pp);
}

void memFreeDouble3(double*** p)
{
	if (p != NULL)
	{
		free(p[0][0]);
		free(p[0]);
		free(p);
		p = NULL;
	}
}

double** memAllocDouble2(int r, int c)
{
	int padding = 10;
	double* a, ** p;
	a = (double*)malloc(sizeof(double) * (r * c + padding));
	//if (a == NULL) { mexErrMsgTxt("memAllocDouble: Memory is too huge.\n"); }
	p = (double**)malloc(sizeof(double*) * r);
	for (int i = 0; i < r; i++) p[i] = &a[i * c];
	return(p);
}
void memFreeDouble2(double** p)
{
	if (p != NULL)
	{
		free(p[0]);
		free(p);
		p = NULL;
	}
}



/*Filter color image*/
void main()
{
	Mat img_1 = imread("D://color.png", IMREAD_COLOR);//

	W = img_1.cols;
	H = img_1.rows;

	mexFunction_2(img_1, img_1);//Filter color image

	waitKey(0);
	system("pause");
}


//
/*這個是測試視差影像(我們的)*/
//void main()
//{
//	Mat img_1 = imread("D://py_disR.png", IMREAD_GRAYSCALE);//
//	//Mat img_2 = imread("D://py_dis_n.png", IMREAD_GRAYSCALE);//second image jump frameIMREAD_GRAYSCALE
//	W = img_1.cols;
//	H = img_1.rows;
//	Mat integral = Mat::zeros(H, W, CV_8UC3);
//	Mat integral2 = Mat::zeros(H, W, CV_8UC3);
//	for (int y = 0; y < H; y++)
//		for (int x = 0; x < W; x++)
//			for (int c = 0; c < 3; c++){
//				integral.at<Vec3b>(y, x)[c] = 40+img_1.at<uchar>(y, x)*2;/*存成3通道，放大數值*/
//				integral2.at<Vec3b>(y, x)[c] = 40 + img_1.at<uchar>(y, x)*2;/*存成3通道，放大數值*/
//			}
//
//	imwrite("iintegralccc_0102.jpg", integral2);
//	mexFunction_2(integral2,integral2);//Filter disparity map 
//
//	waitKey(0);
//	system("pause");
//}

