#include "WeldCircleInspection.h"

int devolpMode;

WeldCircleInspection::WeldCircleInspection(eType type) {
    m_EnumType = type;

    switch (m_EnumType) {
    case eType::ANODE: {
        m_filePath = "C:\\Users\\hong\\Desktop\\hong\\Welding Bead DATA\\anode\\-";
    }
                     break;
    case eType::CATHODE: {
        m_filePath = "C:\\Users\\hong\\Desktop\\hong\\Welding Bead DATA\\cathode\\+";
    }
                       break;

    default: {
        std::cout << "Initialize ERROR !!!\n";
    }
    }

    devolpMode = 0;     //test : 0 or 1
    std::string strImg = m_filePath + "360.bmp";

    m_outputPath = m_filePath.substr(0, m_filePath.size() - 1) + "Test\\";


    m_image = cv::imread(strImg, cv::IMREAD_GRAYSCALE);
    m_colorImage = cv::imread(strImg, cv::IMREAD_COLOR);

    if (m_image.empty()) {
        std::cout << "Image not found!" << std::endl;
        return;
    }
    std::cout << "Image shape: " << m_image.cols << "x" << m_image.rows << std::endl;
    cv::Point minLoc, maxLoc;
    double minVal, maxVal;
    cv::minMaxLoc(m_image, &minVal, &maxVal, &minLoc, &maxLoc);
    std::cout << "Max pixel value: " << maxVal << ", Min pixel value: " << minVal << std::endl;
}

WeldCircleInspection::~WeldCircleInspection() {

}

cv::Mat WeldCircleInspection::resizeShow(cv::Mat mat) {
    cv::Mat resizeMat;
    double scale = 0.3;
    resize(mat, resizeMat, cv::Size(), scale, scale);
    return resizeMat;
}
void WeldCircleInspection::SaveBmp(cv::Mat mat, std::string name) {
    std::string strFile = m_outputPath + name + ".bmp";
    cv::imwrite(strFile, mat);
}
bool WeldCircleInspection::CircleDetectDraw(cv::Mat inputMat, cv::Mat& outputMat, double minDist,
    double param1, double param2,
    int minRadius, int maxRadius) {

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(inputMat, circles,
        cv::HOUGH_GRADIENT, 1, minDist,
        param1, param2,
        minRadius, maxRadius);

    if (circles.empty()) {
        std::cout << "==================================\n";
        std::cout << "circle detec FAILED\n";
        std::cout << "==================================\n";
        return false;
    }

    // circle
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Vec3i c = circles[i];
        int circleX = c[0], circleY = c[1], radius = c[2];
        // circle info copy
        m_cartesianCenter = { circleX, circleY };
        m_nCircumscribedRadius = radius;

        cv::circle(outputMat, cv::Point(circleX, c[1]), radius, cv::Scalar(0, 255, 0), 2);
        cv::circle(outputMat, cv::Point(circleX, c[1]), 2, cv::Scalar(0, 0, 255), 3);

        std::cout << "==================================\n";
        std::cout << "Detect \n";
        std::cout << "x,y : " << circleX << ", " << circleY << " radius : " << radius << "\n";
        std::cout << "==================================\n";
    }

    return true;
}


cv::Mat WeldCircleInspection::AnodeProc(cv::Mat& origImg) {
    cv::Mat outMat;
    outMat = origImg.clone();

    int nIter = 2;
    // test iter Cnt
    for (int i = 0; i < nIter; i++) {
#if 0
        //[ver] First
        // 1. Gaussian Blur
        cv::Mat blur;
        cv::GaussianBlur(outMat, blur, cv::Size(15, 15), 0);


        // 2. CLAHE                            
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));    //clipLimit 2.0,  Size(8,8)
        cv::Mat cl1;
        clahe->apply(blur, cl1);

        outMat = cl1.clone();

        //// test iter Cnt 3
        // for 밖에 for 3
        //cv::Mat median;
        //cv::medianBlur(outMat, median, 5);             // kSize : 23
        //outMat = median.clone();
        //
#else
        // 1. Gaussian Blur
        cv::Mat blur;
        //cv::GaussianBlur(outMat, blur, cv::Size(13, 13), 0);
        cv::GaussianBlur(outMat, blur, cv::Size(15, 15), 0);

        // 2. CLAHE                            
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(4, 4));    //clipLimit 2.0,  Size(8,8)
        cv::Mat cl1;
        clahe->apply(blur, cl1);

        outMat = cl1.clone();
#endif
    }

    if(devolpMode)
    SaveBmp(outMat, "1) [ANODE] pre-Processing Image");

    return outMat;
}

cv::Mat WeldCircleInspection::CathodeProc(cv::Mat& origImg) {

    // cathode orig
    cv::Mat outMat;
    outMat = origImg.clone();

    cv::Mat blur;

    int nIter = 2;
    for (int i = 0; i < nIter; i++) {
        // 1. Gaussian Blur                     cv::Size(11,11)
        cv::GaussianBlur(outMat, blur, cv::Size(11, 11), 0);
        // 2. CLAHE                            clipLimit 1.0,  Size(4,4)
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(4, 4));
        cv::Mat cl1;
        clahe->apply(blur, cl1);
        outMat = cl1.clone();
    }

    if (devolpMode)
    SaveBmp(outMat, "1) [CATHODE] pre-Processing Image");

    
    return outMat;
}

bool WeldCircleInspection::WeldCircleDetect() {
    // [ 1. Pre-processing ]
    TimeStart();
    cv::Mat preProcImg;
    switch (m_EnumType) {
    case eType::ANODE: {
        preProcImg = AnodeProc(m_image);
    }
                     break;
    case eType::CATHODE: {
        preProcImg = CathodeProc(m_image);
    }
                       break;
    default: {
        std::cout << "pre-processing Enum parameter ERROR\n";
        return false;
    }
    }
    TimeEnd("\n\n Processing Time : ");

    // [ 2. Hough Circle Transform (Large Circle detect) ]
    TimeStart();
    double param1 = 50, param2 = 30, minDist = 3000;        // magicNum
    int minRadius = 700, maxRadius = 800;

    if (!CircleDetectDraw(preProcImg, m_colorImage, minDist, param1, param2, minRadius, maxRadius))
        return false;
    TimeEnd("\n\n Circle Inspection Time : ");

    if (devolpMode) {
        SaveBmp(m_colorImage, "2) [houghCl] Image");
        cv::imshow("Hough Circles", resizeShow(m_colorImage));
        cv::waitKey(0);
    }

    // [ 3. Find small Circle ]
    TimeStart();
    if (!SmallCircleDetect(preProcImg))
        return false;
    TimeEnd("\n\n CartesianToPolar Converting Time : ");

    cv::imshow("Result", resizeShow(m_colorImage));
    cv::waitKey(0);

    return true;
}

/* Least-Squares */
std::pair<float, float> fitLineLeastSquares(const std::vector<cv::Point>& points) {
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;

    for (const auto& point : points) {
        sumX += point.x;
        sumY += point.y;
        sumXY += point.x * point.y;
        sumXX += point.x * point.x;
    }

    int n = points.size();

    float m = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
    float b = (sumY - m * sumX) / n;

    // slope, intercept 
    return { m, b };
}


/**
* 최소자승법 外 방법 있나 study
*/
bool WeldCircleInspection::SmallCircleDetect(cv::Mat& _image) {
    cv::Mat image = _image;

    /*
        step 1 : cartesian To polar
        step 2 : polar To Unwrapped image
        step 3 : find edge 
        step 4 : make small Circle
    */
    const int nAngle = 360;
    const int r = m_nCircumscribedRadius;
    const int cx = m_cartesianCenter.x;
    const int cy = m_cartesianCenter.y;

    // 1) Unwrapp (circle -> 2d image)
    int imgWidth = r;
    int imgHeight = nAngle;

    cv::Mat unwrapimage = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);

    for (int height = 0; height < imgHeight; ++height) {
        double theta = height * M_PI / 180.0;  // degree To radian

        for (int width = 0; width < imgWidth; ++width) {
            // 1) cartesian To polar
            int x = (int)(width * cos(theta)) + cx;
            int y = (int)(width * sin(theta)) + cy;

            // 2) Make Unwrapped image
            if (x >= 0 && x < m_image.cols && y >= 0 && y < m_image.rows) {
                int origValue = (int)image.at<uchar>(y, x);
                unwrapimage.at<uchar>(height, width) = static_cast<uchar>(origValue);
            }
        }
    }
    
    //3) find edge
    cv::Mat sobelX, sobelY, magnitudeImage;
    cv::Sobel(unwrapimage, sobelX, CV_8UC1, 1, 0, 3);
    
    int binaryMax = 255;
    cv::Mat binary;
    cv::threshold(sobelX, binary, 40, binaryMax, cv::THRESH_BINARY);                // magicNum
    
    // 3) Edge Inspection
    std::vector<cv::Point>edgePoints;
    
    int avgEdgeX = 0;           /* Least-squares 적용 전 threshold 를 적용하기 위한 avg */ 

    for (int y = 0; y < unwrapimage.rows; y++) {
        int maxEdgeX = -1;
        int maxEdgeValue = -1;

        //----- Range check-----
        for (int x = unwrapimage.cols / 2; x < unwrapimage.cols; x++) {
            int edgeValue = binary.at<uchar>(y, x);

            if (edgeValue == binaryMax) {
                maxEdgeValue = edgeValue;
                maxEdgeX = x;
                break;  //처음 등장하는 Edge 검출
            }
        }
        if (maxEdgeX != -1) {
            edgePoints.push_back(cv::Point(maxEdgeX, y));
            avgEdgeX += maxEdgeX;
        }
    }
    
    // X-axis Edge Threshold
    avgEdgeX /= edgePoints.size();
    int threshold = 3;             // magicNum
    float maxThreshold = avgEdgeX + avgEdgeX*(threshold / 100.0);
    float minThreshold = avgEdgeX - avgEdgeX*(threshold / 100.0);
    
    /* CHECK
        최소 자승법 팀장님께 질문

    // range  :  { (avg) x (-threshold)%    ~   (avg) x (threshold)%  }

    for(int i = 0; i < edgePoints.size(); i ++){
        if (edgePoints[i].x < minThreshold || edgePoints[i].x > maxThreshold) {
            edgePoints.erase(edgePoints.begin() + i);
            i--;
        }
    }
    
    // 4) Line fitting (Least-squares)
    std::pair<float, float> fX = fitLineLeastSquares(edgePoints);
    float slopeY = fX.first;
    float interceptY = fX.second;
    float slopeX = 1.0 / slopeY;
    float interceptX = (-1.0f) * interceptY / slopeY;

    std::cout << "\nedge AVG : " << avgEdgeX;
    std::cout << "\nslope Y : " << slopeY;
    std::cout << "\nintercept Y : " << interceptY;
    std::cout << "\nslope X : " << slopeX;
    std::cout << "\nintercept X : " << interceptX;

    int yStart = 0;
    int yEnd = unwrapimage.rows;
    int xStart = (int)(yStart * slopeX + interceptX);
    int xEnd = (int)(yEnd * slopeX + interceptX);
    
    ////Draw
    cv::Mat resultImage;
    cv::cvtColor(unwrapimage, resultImage, cv::COLOR_GRAY2BGR);
    cv::line(resultImage, cv::Point(xStart, yStart), cv::Point(xEnd, yEnd), cv::Scalar(0, 0, 255));
    */

    /* 
        [Nonhing Least-squares]
        pre-processing image Circle unwrap -> Sobel -> binary -> find EdgeX average (small circle radius)
    */

    // 4) make small Circle
    cv::Mat resultImage;
    cv::cvtColor(unwrapimage, resultImage, cv::COLOR_GRAY2BGR);
    cv::line(resultImage, cv::Point(avgEdgeX, 0), cv::Point(avgEdgeX, unwrapimage.rows), cv::Scalar(0, 0, 255), 1);
    
    // draw Circle
    m_nInscribedRadius = avgEdgeX;
    cv::circle(m_colorImage, m_cartesianCenter, m_nInscribedRadius, cv::Scalar(0, 0, 255), 2);
    m_nWeldWidth = m_nCircumscribedRadius - m_nInscribedRadius;
    std::cout << "\n Weld Width : " << m_nWeldWidth <<"pixel";

    if (devolpMode) {
        SaveBmp(unwrapimage, "3.1) Unwrapped image");
        SaveBmp(sobelX, "3.2) sobel");
        SaveBmp(binary, "3.3) binary");
        SaveBmp(resultImage, "3.4) Least-Squares Img");
        SaveBmp(m_colorImage, "3.5) Welding Circle Inspection");
        cv::imshow("sss", resultImage);
        cv::waitKey(0);
        cv::imshow("circle Inspection", resizeShow(m_colorImage));
        cv::waitKey(0);
    }

    /*
    * Unwraapped image  ->  Sobel, binary value 분포를 활용한 Edge detect
    * 
    cv::Mat sobelX;
    cv::Sobel(unwrapimage, sobelX, CV_8UC1, 1, 0, 3);
    SaveBmp(sobelX, "sobelX");
    cv::imshow("sobelX", sobelX);
    cv::waitKey(0);

    cv::Mat binary;
    cv::threshold(sobelX, binary, 50, 255, cv::THRESH_BINARY);
    SaveBmp(binary, "binary");
    cv::imshow("binary", binary);
    cv::waitKey(0);


    std::vector<int> xCount(binary.cols, 0);
    for (int y = 0; y < binary.rows; y++) {
        for (int x = 0; x < binary.cols; x++) {
            if (binary.at<uchar>(y, x) == 255) {
                xCount[x]++;
            }
        }
    }
    
    int maxCount = 0;
    int maxX = 0;
    for (int j = 0; j < xCount.size(); ++j) {
        if (xCount[j] > maxCount) {
            maxCount = xCount[j];
            maxX = j;
        }
    }

    cv::line(unwrapimage, cv::Point(maxX, 0), cv::Point(maxX, unwrapimage.rows - 1), cv::Scalar(255), 1);
    cv::imshow("Unwrapped Circle image", unwrapimage);
    cv::waitKey(0);
    SaveBmp(unwrapimage, "Unwrapped Result");*/
    return true;
}


void WeldCircleInspection::TimeEnd(std::string str) {
    m_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time);
    std::cout << str << duration.count() << " ms" << std::endl;
}
