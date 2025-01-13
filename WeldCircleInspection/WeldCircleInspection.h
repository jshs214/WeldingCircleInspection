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
	--> ANODE, CATHODE �� 1�� ��� ��
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
	void SaveBmp(cv::Mat mat, std::string name);

	// algorithm
	// [pre -Processing]
	cv::Mat AnodeProc(cv::Mat& origImg);
	cv::Mat CathodeProc(cv::Mat& origImg);
	bool LargeCircleDetect(cv::Mat inputMat, cv::Mat& outputMat, double minDist,
		double param1, double param2,
		int minRadius, int maxRadius);
	bool SmallCircleDetect(cv::Mat& image);
	std::pair<float, float> LeastSquares(const std::vector<cv::Point>& points);
	std::pair<float, float> Ransac(std::vector<cv::Point>& points, int maxIterations,
		float threshold, int& inlierCount);

private:
	cv::Mat m_image;		//gray-scale
	cv::Mat m_colorImage;	//3ch

	// circle Info
	int m_nCircumscribedRadius;				//���� radius
	std::vector<std::pair<int, float>> m_vInscribed;		// radian, radius
	std::vector<std::pair<int, float>> m_vWeldBeadWidth;	// radian, welding Width
	float m_fMinWeldWidth =INT_MAX , m_fMaxWeldWidth = INT_MIN;

	cv::Point2i m_cartesianCenter;


	// For Utility 
	eType m_EnumType;
	std::string m_filePath;
	std::string m_outputPath;

	// For timer
	std::chrono::high_resolution_clock::time_point m_start_time;
	std::chrono::high_resolution_clock::time_point m_end_time;
	void TimeStart() { m_start_time = std::chrono::high_resolution_clock::now(); }
	void TimeEnd(std::string str);
	long long llWeldCircleDetectTime;
};

