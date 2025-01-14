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
	--> ANODE, CATHODE ñé 1°³ »ç¿ë Áß
*/
#pragma once
class WeldCircleInspection
{
public:
	WeldCircleInspection(eType type);
	~WeldCircleInspection();

	// Main logic
	bool WeldCircleDetect();	

private:
	// For Utility
	bool ReadImage(std::string path);
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
	bool SmallCircleDetect(cv::Mat& image);
	std::pair<float, float> Ransac(std::vector<cv::Point>& points, int maxIterations, float threshold, int& inlierCount);
	std::pair<float, float> LeastSquares(const std::vector<cv::Point>& points);

	bool __SmallCircleDetect(cv::Mat& image);
private:
	cv::Mat m_image;		//gray-scale
	cv::Mat m_colorImage;	//3ch

	// circle Info
	cv::Point2i m_cartesianCenter;
	// Circumscribed Circle
	int m_nCircumscribedRadius;								
	
	// Inscribed Circle
	// 3 o'clock -> degree 0
	// 9 o'clock -> degree 180
	std::vector<std::pair<double, float>> m_vInscribed;		// radian, radius
	std::vector<std::pair<double, float>> m_vWeldBeadWidth;	// radian, welding Width

	float m_fMinWeldWidth =INT_MAX , m_fMaxWeldWidth = INT_MIN;		//test

	// For Utility 
	eType m_EnumType;
	std::string m_filePath;
	std::string m_outputPath;

	// For time
	std::chrono::high_resolution_clock::time_point m_start_time;
	std::chrono::high_resolution_clock::time_point m_end_time;
	void TimeStart() { m_start_time = std::chrono::high_resolution_clock::now(); }
	void TimeEnd(std::string str);
	long long llWeldCircleDetectTime;

};

