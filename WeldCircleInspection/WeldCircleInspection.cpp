#include "WeldCircleInspection.h"

int developMode ;
#define THICKNESS 3
WeldCircleInspection::WeldCircleInspection() {
    
    llWeldCircleDetectTime = 0;
    
}

WeldCircleInspection::~WeldCircleInspection() {

}

bool WeldCircleInspection::WeldCircleDetect() {
    /* hardcoding */
    developMode = false;

    //// cathode Albeodo
    m_EnumType = eType::CATHODE;
    std::string path = m_filePath + "Result [2024-07-22]19.32.44.247_0_1_0.bmp";

    //// Anode Albeodo
    /*m_EnumType = eType::ANODE;
    std::string path = m_filePath + "Result [2024-07-22]19.31.02.262_0_0_0.bmp";*/
    

    // [ 0. Read]
    TimeStart();
    if (!ReadImage(path))
        return false;
    TimeEnd("\n\n 0) Read Image ");


    // [ 1. Pre-processing ]
    TimeStart();
    cv::Mat preProcImg;

    if (m_EnumType == eType::ANODE)
        preProcImg = AnodeProc(m_image);
    else if (m_EnumType == eType::CATHODE)
        preProcImg = CathodeProc(m_image);
    else {
        std::cout << "weld image type check !\n";
        return false;
    }

    if (developMode)
        SaveBmp("1) pre-Processing Image", preProcImg);
    TimeEnd("\n\n 1) Processing Time : ");

    // [ 2. Hough Circle Transform (Large Circle detect) ]
    TimeStart();
    double param1 = 130, param2 = 20;       // scale 0.5

    int nMinDist = std::min(m_image.cols, m_image.rows);
    double minDist = (double)nMinDist;

    int minRadius = 700, maxRadius = 800;
    
    if (!LargeCircleDetect(preProcImg, m_colorImage, minDist, param1, param2, minRadius, maxRadius))
        return false;
    if (developMode) {
        SaveBmp("2) [houghCl] Image", m_colorImage);
        cv::imshow("Hough Circles", resizeShow(m_colorImage));
        cv::waitKey(0);
    }
    TimeEnd("\n\n 2) Large Circle Inspection Time : ");

    // [ 3. Find small Circle ]
    TimeStart();
    //if (!SmallCircleDetect(preProcImg))
    if (!__SmallCircleDetect(preProcImg))
        return false;
    TimeEnd("\n\n 3) Small Circle Inspection Time : ");

    // check Total Time
    std::cout << "\n\n [Total Detecting Time] " << llWeldCircleDetectTime << "ms\n";
    
    // [4. weld Circle area ]
    if (!CalculateCircleBead()) {
        return false;
    }

    cv::imshow("Result", resizeShow(m_colorImage));
    cv::waitKey(0);

    return true;
}
cv::Mat WeldCircleInspection::resizeShow(cv::Mat mat) {
    cv::Mat resizeMat;
    double scale = 0.3;
    resize(mat, resizeMat, cv::Size(), scale, scale);
    return resizeMat;
}
void WeldCircleInspection::SaveBmp(std::string name, cv::Mat mat) {
    std::string strFile = m_outputPath + name + ".bmp";
    cv::imwrite(strFile, mat);
}

bool WeldCircleInspection::LargeCircleDetect(cv::Mat inputMat, cv::Mat& outputMat, double minDist,
    double param1, double param2,
    int minRadius, int maxRadius) {

     // Down-scale
    double scaleFactor = 0.5;  // magnification
    int scaleMinRadius = (double)minRadius * scaleFactor;
    int scaleMaxRadius = (double)maxRadius * scaleFactor;
    int scaleWidth = m_image.cols * scaleFactor;
    int scaleHeight = m_image.rows * scaleFactor;
    double scaleMinDist = minDist * scaleFactor;

    cv::Mat resizedImage;
    cv::resize(inputMat, resizedImage, cv::Size(scaleWidth, scaleHeight), scaleFactor, scaleFactor, cv::INTER_AREA);
    //SaveBmp("resizeImage", resizedImage);

    // Circle Detecting from Down-scale Image
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(resizedImage, circles, cv::HOUGH_GRADIENT, 1, scaleMinDist, param1, param2,
        scaleMinRadius, scaleMaxRadius);
    
    if (circles.empty()) {
        std::cout << "==================================\n";
        std::cout << "Hough circle detect FAILED\n";
        std::cout << "==================================\n";
        return false;
    }

    for (size_t i = 0; i < circles.size(); i++) {
        cv::Vec3i c = circles[i];
        int circleX = c[0], circleY = c[1], radius = c[2];

        // up-scale
        cv::Point originalCenter(cvRound(circleX * (1 / scaleFactor)), cvRound(circleY * (1 / scaleFactor)));
        int originalRadius = cvRound(radius * (1 / scaleFactor));

        m_cartesianCenter = { originalCenter.x, originalCenter.y };
        m_nOuterRadius = originalRadius;

        // 원의 중심과 반지름을 원본 이미지에 맞게 보정 후 그리기
        cv::circle(outputMat, originalCenter, originalRadius, cv::Scalar(255, 0, 0), THICKNESS); // 원
        cv::circle(outputMat, originalCenter, 2, cv::Scalar(0, 0, 255), THICKNESS);  // 중심점
    }
    std::cout << m_cartesianCenter.x << "," << m_cartesianCenter.y << "\n";

    return true;
}


cv::Mat WeldCircleInspection::AnodeProc(cv::Mat& origImg) {
    cv::Mat outMat;
    outMat = origImg.clone();

    int nIter = 2;
    // test iter Cnt
    for (int i = 0; i < nIter; i++) {
        // 1. Gaussian Blur
        cv::Mat blur;
        //cv::GaussianBlur(outMat, blur, cv::Size(13, 13), 0);
        cv::GaussianBlur(outMat, blur, cv::Size(15, 15), 0);

        // 2. CLAHE                            
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(4, 4));    //clipLimit 2.0,  Size(8,8)
        cv::Mat cl1;
        clahe->apply(blur, cl1);

        outMat = cl1.clone();
    }

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

    return outMat;
}

/* Least-Squares 
 * @return : slope_Y, intercept_Y */
std::pair<float, float> WeldCircleInspection::LeastSquares(const std::vector<cv::Point>& points) {
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

/* ransac : remove outlier
 * @return : slope_X, intercept_X*/
std::pair<float, float> WeldCircleInspection::Ransac(std::vector<cv::Point>& points, int maxIterations,
    float threshold, int& inlierCount) {
    if (points.empty()) {
        return { -1, -1 };
    }

    srand(time(0));  // random seed

    int bestInlierCount = 0;
    std::pair<float, float> bestLine;

    // 반복적으로 두 점을 랜덤하게 선택하여 모델을 추정
    for (int i = 0; i < maxIterations; ++i) {
        int idx1 = rand() % points.size();
        int idx2 = rand() % points.size();

        // 두 점이 같지 않도록 확인
        if (idx1 == idx2) continue;

        cv::Point p1 = points[idx1];
        cv::Point p2 = points[idx2];

        std::pair<float,float> fX = LeastSquares({ p1, p2 });

        // fit Line  Least-Squares 
        // return type : y 방정식
        float slopeY = fX.first; 
        float interceptY = fX.second;

        // 기울기와 절편을 X 기준으로 계산
        float slopeX = 1.0f / slopeY;
        float interceptX = (-1.0f) * interceptY / slopeY;

        // InlierCount
        int count = 0;
        for (const auto& point : points) {
            float x_pred = slopeX * point.y + interceptX;

            // 점과 직선 사이의 거리 계산
            float distance = std::abs(point.x - x_pred);

            if (distance < threshold) {             
                count++;
            }
        }

        // 가장 많은 인라이어를 가진 모델 선택
        if (count > bestInlierCount) {
            bestInlierCount = count;
            bestLine = { slopeX, interceptX };
        }
    }

    inlierCount = bestInlierCount;
    return bestLine;
}

/*
    * [Developing--]
        pre-processing image Circle unwrap -> Sobel -> binary
        -> EdgeX point -> RANSAN -> Least Squares

        step 1 : convert  circle -> 2d unwrapped image
        step 2 : unwrapped image To binary image
        step 3 : Width-axis Edge inspection
        step 4 : make small Circle
*/

bool WeldCircleInspection::__SmallCircleDetect(cv::Mat& _image) {
    cv::Mat image = _image;

    // Outer circle Info
    cv::Point2i center(m_cartesianCenter.x, m_cartesianCenter.y);
    int nRadius = m_nOuterRadius;
    const int cx = m_cartesianCenter.x;
    const int cy = m_cartesianCenter.y;

    /*
    * polarImage -> (r * 360) 생성
    * Width  : r
    * Height : 0~ 360 (degree)까지 균등 분포
    */
    int polarImageWidth = m_nOuterRadius;
    int polarImageHeight = m_image.rows;

    /* Polar Image Edge 검출 시 ROI*/
    int nMinEdgeX = 400, nMaxEdgeX = 500;

    // 1) Circle -> unwrapimage
    cv::Size dSize(polarImageWidth, polarImageHeight);
    cv::Mat polarImage;
    cv::warpPolar(image, polarImage, dSize, center, nRadius, cv::INTER_LINEAR);
 
    //2) unwrapimage To binary
    cv::Mat sobelX;
    cv::Sobel(polarImage, sobelX, CV_8UC1, 1, 0, 3);

   // 3) Width-axis Edge inspection
    std::vector<cv::Point>edgePoints;

    for (int y = 0; y < polarImageHeight; y++) {
        int nXcoord = -1;
        int nXmaxval= -1;

        // Range-hardcoding
        for (int x = nMinEdgeX; x < nMaxEdgeX; x++) {
            int val = sobelX.at<uchar>(y, x);
            if (val > nXmaxval) {
                nXcoord = x;
                nXmaxval = val;
            }
        }
        edgePoints.push_back({ nXcoord, y });
    }

    // 4) make small Circle
    int maxIterations = 200000;         // iterator
    float ransacThreshold = 0.03f;      // Inlier Threshold
    int inlierCount = 0;

    // 4-1) ransac & Least-Sqaures
    std::pair<float, float> fX = Ransac(edgePoints, maxIterations, ransacThreshold, inlierCount);
    if (fX.first == -1 && fX.second == -1) {
        std::cout << "Ransac Error \n";
        return false;
    }

    /* 추출된 직선의 기울기, 절편 */
    float slopeX = fX.first;
    float interceptX = fX.second;

    std::cout << slopeX << "," << interceptX << "\n";
    
    // 내부 원 좌표 계산 Polar
    for (int y = 0; y< polarImageHeight; y++) {
        float fRealDegree = (y / (double)polarImageHeight) * 360.0;     // interpolation Roll-back
        // 직선의 방정식 y = mX + b
        float fPolar_Radian= (fRealDegree * M_PI) / 180.0;               // degree To radian

        float fPolar_InnerRadius = y * slopeX + interceptX;
        float fPolar_OuterRadius = nRadius;

        m_vecPolarInner.push_back({ fPolar_Radian, fPolar_InnerRadius });
        m_vecPolarOuter.push_back({ fPolar_Radian, fPolar_OuterRadius });
    }
    
    // 내부 원 좌표 변환 Polar to cartesian
    if (m_vecPolarInner.size() != m_vecPolarOuter.size())
        return false;

    for (int i = 0; i < m_vecPolarInner.size(); i ++) {
        double radian = m_vecPolarInner[i].first;

        float fInnerRadius = m_vecPolarInner[i].second;
        float fOuterRadius = m_vecPolarOuter[i].second;

        /* cartesian에서 내부 원 좌표 */
        int nCartesianInner_x = (fInnerRadius * cos(radian)) + cx;
        int nCartesianInner_y = (fInnerRadius * sin(radian)) + cy;

        int nCartesianOuter_x = (fOuterRadius * cos(radian)) + cx;
        int nCartesianOuter_y = (fOuterRadius * sin(radian)) + cy;

        //double degree = radian * (180.0 / M_PI);
        m_vecCartesianInner.push_back({ nCartesianInner_x, nCartesianInner_y });
        m_vecCartesianOuter.push_back({ nCartesianOuter_x, nCartesianOuter_y });
    }


    // 용접 비드 사이즈 계산
    std::set<std::pair<int, int>> setWeldBead_pos;  // 중간 점을 저장할 set
    for (int i = 0; i < m_vecCartesianInner.size(); i++) {
        int x1 = m_vecCartesianInner[i].first;
        int y1 = m_vecCartesianInner[i].second;
        int x2 = m_vecCartesianOuter[i].first;
        int y2 = m_vecCartesianOuter[i].second;

        // 두 점 사이의 실제 유클리드 거리 계산
        float distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));

        // 두 점 사이의 간격을 기준으로 중간 점을 구함
        int numPointsBetween = round(distance);  // 거리만큼 점의 개수를 구함

        for (int j = 0; j <= numPointsBetween; ++j) {
            float t = j / (float)numPointsBetween;  // 보간 비율
            int intermediate_x = (int)(x1 + t * (x2 - x1));
            int intermediate_y = (int)(y1 + t * (y2 - y1));

            setWeldBead_pos.insert({ intermediate_x, intermediate_y });
            cv::circle(m_colorImage, cv::Point(intermediate_x, intermediate_y), 1, cv::Scalar(0, 122, 122), 1);
        }
    }

    m_nWeldingBeadPixelCnt = setWeldBead_pos.size();

    // Draw
    cv::Mat resultImage;
    cv::cvtColor(polarImage, resultImage, cv::COLOR_GRAY2BGR);

    float yStart_X = 0 * slopeX + interceptX;                     // if(y ==0) -> x 좌표
    float yEnd_X = (polarImageHeight)*slopeX + interceptX;        // if(y== height) -> x 좌표

    /* cartesian Image */
    for (auto i : m_vecCartesianInner) {
        cv::circle(m_colorImage, cv::Point(i.first, i.second), 1, cv::Scalar(0, 0, 255), THICKNESS);
    }
    
    /*  polar Image  */
    /* Least-Squares Line */
    cv::line(resultImage, cv::Point(yStart_X, 0), cv::Point(yEnd_X, polarImageHeight), cv::Scalar(255, 0, 0), THICKNESS);
    /* Edge Detect Point*/
    for (auto i : edgePoints)
        cv::circle(resultImage, cv::Point(i.x, i.y), 1, cv::Scalar(0, 0, 255), 1);


    if (developMode) {
        SaveBmp("3.1) Unwrapped image", polarImage);
        SaveBmp("3.2) sobel", sobelX);
        SaveBmp("3.3) Least-Squares Img", resultImage);
        SaveBmp("4) Welding Circle Inspection", m_colorImage);
    }
    else {
        SaveBmp("3.3) Least-Squares Img", resultImage);
        SaveBmp("4) Welding Circle Inspection", m_colorImage);
    }


    return true;
}

void WeldCircleInspection::TimeEnd(std::string str) {
    m_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time);
    std::cout << str << duration.count() << " ms" << std::endl;
    llWeldCircleDetectTime += duration.count();
}

bool WeldCircleInspection::ReadImage(std::string strImg) {
    // hardcoding
    switch (m_EnumType) {
    case eType::ANODE: {
        m_filePath = "C:\\Users\\hong\\Desktop\\hong\\Welding Bead DATA\\anode\\";
    }
                     break;
    case eType::CATHODE: {
        m_filePath = "C:\\Users\\hong\\Desktop\\hong\\Welding Bead DATA\\cathode\\";
    }
                       break;
    default: {
        std::cout << "Initialize ERROR !!!\n";
    }
           break;
    }

    //m_outputPath = m_filePath.substr(0, m_filePath.size() - 1) + "Test\\";
    m_outputPath = m_filePath + "Test\\";
    strImg = m_filePath + strImg;


    m_image = cv::imread(strImg, cv::IMREAD_GRAYSCALE);
    m_colorImage = cv::imread(strImg, cv::IMREAD_COLOR);


    if (m_image.empty()) {
        std::cout << "Image not found!" << std::endl;
        return false;
    }
    if (m_colorImage.empty()) {
        return false;
    }

    std::cout << "Image shape: " << m_image.cols << "x" << m_image.rows << std::endl;
    /*cv::Point minLoc, maxLoc;
    double minVal, maxVal;
    cv::minMaxLoc(m_image, &minVal, &maxVal, &minLoc, &maxLoc);
    std::cout << "Max pixel value: " << maxVal << ", Min pixel value: " << minVal << std::endl;*/
    return true;
}

bool WeldCircleInspection::CalculateCircleBead() {
    std::cout << "Welding Bead Area : " << m_nWeldingBeadPixelCnt << " pixels\n";

    return true;
}
