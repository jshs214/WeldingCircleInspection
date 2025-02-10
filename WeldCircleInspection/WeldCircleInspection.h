#include <iostream>
#include <string>
#include <chrono>
#define _USE_MATH_DEFINES	// for M_PI
#include <cmath>
#include <numeric>
#include <map>
#include "opencv2/opencv.hpp"

enum class eType
{
	None = -1,
	ANODE,
	CATHODE,
	eTypeSIZE,
};

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

