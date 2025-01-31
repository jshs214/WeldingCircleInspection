#include <iostream>
#include <string>
#include <chrono>
#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <numeric>
#include <map>
#include "opencv2/opencv.hpp"

enum class eType
{
	None = -1,
	ANODE,
	CATHODE,
	LoadVoxelData,
};

/*
	* [Developing--]
		pre-processing image Circle unwrap -> Sobel -> binary
		-> EdgeX point -> RANSAN -> Least Squares

		step 1 : convert  circle -> 2d unwrapped image
		step 2 : unwrapped image To binary image
		step 3 : Width-axis Edge inspection
		step 4 : make small Circle
*/
/*
	--> ANODE, CATHODE ñé 1°³ »ç¿ë Áß
*/
#pragma once
class WeldCircleInspection
{
public:
	WeldCircleInspection();
	~WeldCircleInspection();

	// Main logic
	bool WeldCircleDetect();	

private:
	// For Utility
	bool ReadImage();
	cv::Mat resizeShow(cv::Mat mat);
	void SaveBmp(std::string name, cv::Mat mat);

	// algorithm
	// [pre -Processing]
	cv::Mat AnodeProc(cv::Mat& origImg);
	cv::Mat CathodeProc(cv::Mat& origImg);

	// [based Hough Transform]
	bool LargeCircleDetect(cv::Mat inputMat, cv::Mat& outputMat, double minDist,
		double param1, double param2,
		int minRadius, int maxRadius);
	// [circle To 2d-image]
	bool __SmallCircleDetect(cv::Mat& image);	// cv Polar
	cv::Point2f Ransac(std::vector<cv::Point>& points, int maxIterations, float threshold, int& inlierCount);
	cv::Point2f LeastSquares(const std::vector<cv::Point>& points);
	bool CalculateCircleBead();
	
private:
	cv::Mat m_image;		//gray-scale
	cv::Mat m_colorImage;	//3ch

	// circle Info
	cv::Point2i m_cartesianCenter;
	// Circumscribed Circle
	int m_nOuterRadius;								
	
	// Inscribed Circle
	// 3 o'clock -> degree 0
	// 9 o'clock -> degree 180
	std::vector<cv::Point2f> m_vecPolarInner;		// radian, radius
	std::vector<cv::Point2f> m_vecPolarOuter;		// radian, radius
	std::vector<cv::Point2i> m_vecCartesianInner;		//x,y
	std::vector<cv::Point2i> m_vecCartesianOuter;		//x,y

	int m_nWeldingBeadPixelCnt;

	// For Utility 
	eType m_EnumType;
	std::string m_filePath;
	std::string m_fileName;
	std::string m_outputPath;

	// For time
	std::chrono::high_resolution_clock::time_point m_start_time;
	std::chrono::high_resolution_clock::time_point m_end_time;
	void TimeStart() { m_start_time = std::chrono::high_resolution_clock::now(); }
	void TimeEnd(std::string str);
	long long llWeldCircleDetectTime;
};

