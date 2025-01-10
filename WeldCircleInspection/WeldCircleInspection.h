#include <iostream>
#include <string>
#include <chrono>
#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <numeric>
#include "opencv2/opencv.hpp"


enum class eType
{
	None = -1,
	ANODE,
	CATHODE,
	LoadVoxelData,
};

/*
	--> ANODE, CATHODE 中 1개 사용 중
*/

#pragma once
class WeldCircleInspection
{
public:
	WeldCircleInspection(eType type);
	~WeldCircleInspection();

	// pre -Processing
	cv::Mat AnodeProc(cv::Mat& origImg);
	cv::Mat CathodeProc(cv::Mat& origImg);

	// Main logic - Circle Detect Function
	bool WeldCircleDetect();	

	bool SmallCircleDetect(cv::Mat& image);

private:
	// For Utility
	cv::Mat resizeShow(cv::Mat mat);
	void SaveBmp(cv::Mat mat, std::string name);

	// algorithm
	bool CircleDetectDraw(cv::Mat inputMat, cv::Mat& outputMat, double minDist,
		double param1, double param2,
		int minRadius, int maxRadius);
	
private:
	cv::Mat m_image;		//gray-scale
	cv::Mat m_colorImage;	//3ch

	// circle Info
	int m_nCircumscribedRadius;				//외접 radius
	int m_nInscribedRadius;					//내접 radius
	cv::Point2i m_cartesianCenter;

	int m_nWeldWidth;


	// For Utility 
	eType m_EnumType;
	std::string m_filePath;
	std::string m_outputPath;

	//timer
	std::chrono::high_resolution_clock::time_point m_start_time;
	std::chrono::high_resolution_clock::time_point m_end_time;

	void TimeStart() { m_start_time = std::chrono::high_resolution_clock::now(); }
	void TimeEnd(std::string str);
};

