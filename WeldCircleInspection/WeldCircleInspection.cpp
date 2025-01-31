#include "WeldCircleInspection.h"

int developMode ;
#define THICKNESS 3

/* Image Size 고정 되면 파라미터 최적화*/
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
    //// Anode Albeodo
    //m_EnumType = eType::ANODE;
    
    // [ 0. Read]
    TimeStart();
    if (!ReadImage())
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

    int minRadius, maxRadius;
    if (m_image.cols > 5000) {
        minRadius = 1150;
        maxRadius = 1250;
    }else{
        minRadius = 700;
        maxRadius = 800;
    }

    
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
    if (!__SmallCircleDetect(preProcImg))
        return false;
    TimeEnd("\n\n 3) Small Circle Inspection Time : ");

    // check Total Time
    std::cout << "\n\n [Total Detecting Time] " << llWeldCircleDetectTime << "ms\n";
    
    // [4. weld Circle area ]
    if (!CalculateCircleBead()) {
        return false;
    }

    SaveBmp("image.bmp", m_image);
    SaveBmp("4) Welding Circle Inspection", m_colorImage);

    
    return true;
}
cv::Mat WeldCircleInspection::resizeShow(cv::Mat mat) {
    cv::Mat resizeMat;
    float scale = 0.3;
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
    
    // Circle Detecting Down-scale Image
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

        // draw - origin image
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
        cv::GaussianBlur(outMat, blur, cv::Size(15, 15), 0);

        // 2. CLAHE                            
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(4, 4));
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
cv::Point2f WeldCircleInspection::LeastSquares(const std::vector<cv::Point>& points) {
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
cv::Point2f WeldCircleInspection::Ransac(std::vector<cv::Point>& points, int maxIterations,
    float threshold, int& inlierCount) {
    if (points.empty()) {
        return { -1, -1 };
    }

    srand(time(0));  // random seed

    int bestInlierCount = 0;
    cv::Point2f bestLine;

    // 반복적으로 두 점을 랜덤하게 선택하여 모델을 추정
    for (int i = 0; i < maxIterations; ++i) {
        int idx1 = rand() % points.size();
        int idx2 = rand() % points.size();

        // 두 점이 같지 않도록 확인
        if (idx1 == idx2) continue;

        cv::Point p1 = points[idx1];
        cv::Point p2 = points[idx2];

        cv::Point2f fX = LeastSquares({ p1, p2 });

        // fit Line  Least-Squares 
        // return type : y 방정식
        float slopeY = fX.x; 
        float interceptY = fX.y;

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
    * Height : degree
    */
    int polarImageWidth = m_nOuterRadius;
    int polarImageHeight = m_image.rows;

    /* Polar Image Edge 검출 시 ROI*/
    int nMinEdgeX, nMaxEdgeX;
    if (m_image.cols > 5000) {
        nMinEdgeX = 600, nMaxEdgeX = 750;
    }
    else {
        nMinEdgeX = 400, nMaxEdgeX = 500;
    }
    

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
    int maxIterations = 100000;         // iterator
    float ransacThreshold = 0.05f;      // Inlier Threshold
    int inlierCount = 0;

    // 4-1) ransac & Least-Sqaures
    cv::Point2f fX = Ransac(edgePoints, maxIterations, ransacThreshold, inlierCount);
    if (fX.x == -1 && fX.y == -1) {
        std::cout << "Ransac Error \n";
        return false;
    }

    /* 추출된 직선의 기울기, 절편 */
    float slopeX = fX.x;
    float interceptX = fX.y;

    // 4-2) 내부 원 좌표 계산 - Polar
    for (int y = 0; y< polarImageHeight; y++) {
        float fRealDegree = (y / (float)polarImageHeight) * 360.0;     // interpolation Roll-back
        // 직선의 방정식 y = mX + b
        float fPolar_Radian= (fRealDegree * M_PI) / 180.0;               // degree To radian

        float fPolar_InnerRadius = y * slopeX + interceptX;
        float fPolar_OuterRadius = nRadius;

        m_vecPolarInner.push_back({ fPolar_Radian, fPolar_InnerRadius });
        m_vecPolarOuter.push_back({ fPolar_Radian, fPolar_OuterRadius });
    }
    
    // 5. 내부 원 좌표 변환 Polar to cartesian
    if (m_vecPolarInner.size() != m_vecPolarOuter.size() || m_vecPolarInner.empty() || m_vecPolarOuter.empty())
        return false;

    for (int i = 0; i < m_vecPolarInner.size(); i ++) {
        float radian = m_vecPolarInner[i].x;

        float fInnerRadius = m_vecPolarInner[i].y;
        float fOuterRadius = m_vecPolarOuter[i].y;

        /* cartesian에서 내부 원 좌표 */
        int nCartesianInner_x = (fInnerRadius * cos(radian)) + cx;
        int nCartesianInner_y = (fInnerRadius * sin(radian)) + cy;

        int nCartesianOuter_x = (fOuterRadius * cos(radian)) + cx;
        int nCartesianOuter_y = (fOuterRadius * sin(radian)) + cy;

        //float degree = radian * (180.0 / M_PI);
        m_vecCartesianInner.push_back({ nCartesianInner_x, nCartesianInner_y });
        m_vecCartesianOuter.push_back({ nCartesianOuter_x, nCartesianOuter_y });
    }


    // 5-1) 용접 비드 사이즈 계산
    cv::Mat weldingBead_mask = cv::Mat::zeros(m_image.rows, m_image.cols, CV_8U);

    for (int i = 0; i < m_vecCartesianInner.size(); i++) {
        int x1 = m_vecCartesianInner[i].x;
        int y1 = m_vecCartesianInner[i].y;

        int x2 = m_vecCartesianOuter[i].x;
        int y2 = m_vecCartesianOuter[i].y;

        float distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));

        // Inner,Outer Circle Point 거리를 기준으로 사이 pixel를 구함
        int numPointsBetween = round(distance);

        for (int j = 0; j <= numPointsBetween; ++j) {
            int intermediate_x = ((numPointsBetween - j) * x1 + j * x2) / numPointsBetween;
            int intermediate_y = ((numPointsBetween - j) * y1 + j * y2) / numPointsBetween;

            cv::Point xy_ptCartesian(intermediate_x, intermediate_y);

            if (weldingBead_mask.at<uchar>(xy_ptCartesian)) continue;

            weldingBead_mask.at<uchar>(xy_ptCartesian) = 255;
            
        }
    }

    // 5-2 cartesian image Welding area interpolation
    cv::Mat dilate_mask;
    cv::Mat dilate_kernel= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(weldingBead_mask, dilate_mask, dilate_kernel);

    m_nWeldingBeadPixelCnt = cv::countNonZero(weldingBead_mask);

    // Draw
    cv::Mat resultImage;
    cv::cvtColor(polarImage, resultImage, cv::COLOR_GRAY2BGR);
    
    /* cartesian Image */
    m_colorImage.setTo(cv::Scalar(0, 77, 77), dilate_mask);

    for (int i = 0; i < m_vecCartesianInner.size(); i++) {
        cv::Point ptInner(m_vecCartesianInner[i].x, m_vecCartesianInner[i].y);
        cv::Point ptOuter(m_vecCartesianOuter[i].x, m_vecCartesianOuter[i].y);

        cv::circle(m_colorImage, ptInner, 1, cv::Scalar(0, 0, 255), THICKNESS);
        cv::circle(m_colorImage, ptOuter, 1, cv::Scalar(255, 0, 0), THICKNESS);
    }

    /*  polar Image  */
    float yStart_X = 0 * slopeX + interceptX;                     // if(y ==0) -> x 좌표
    float yEnd_X = (polarImageHeight)*slopeX + interceptX;        // if(y== height) -> x 좌표
    /* Least-Squares Line */
    cv::line(resultImage, cv::Point(yStart_X, 0), cv::Point(yEnd_X, polarImageHeight), cv::Scalar(255, 0, 0), THICKNESS);
    /* Edge Detect Point*/
    for (auto i : edgePoints)
        cv::circle(resultImage, cv::Point(i.x, i.y), 1, cv::Scalar(0, 0, 255), 1);


    if (developMode) {
        SaveBmp("3.1) Unwrapped image", polarImage);
        SaveBmp("3.2) sobel", sobelX);
        SaveBmp("3.3) Least-Squares Img", resultImage);
    }
    else {
        SaveBmp("3.3) Least-Squares Img", resultImage);
    }


    return true;
}

void WeldCircleInspection::TimeEnd(std::string str) {
    m_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(m_end_time - m_start_time);
    std::cout << str << duration.count() << " ms" << std::endl;
    llWeldCircleDetectTime += duration.count();
}

bool WeldCircleInspection::ReadImage() {
    // hardcoding
    switch (m_EnumType) {
    case eType::ANODE: {
        //m_filePath = "C:\\Users\\hong\\Desktop\\hong\\Welding Bead DATA\\anode\\";
        m_filePath = "C:\\Users\\hong\\Desktop\\hong\\Welding Bead DATA\\";
        m_fileName = "ai Result [2025-01-21]17.53.42.741_0_0_0.bmp";
    }
                     break;
    case eType::CATHODE: {
        //m_filePath = "C:\\Users\\hong\\Desktop\\hong\\Welding Bead DATA\\cathode\\";
        m_filePath = "C:\\Users\\hong\\Desktop\\hong\\Welding Bead DATA\\";
        m_fileName = "cu Result [2025-01-21]17.42.40.092_0_0_0.bmp";
    }
                       break;
    default: {
        std::cout << "Initialize ERROR !!!\n";
    }
           break;
    }

    //m_outputPath = m_filePath.substr(0, m_filePath.size() - 1) + "Test\\";
    m_outputPath = m_filePath + "Test\\";
    std::string path = m_filePath + m_fileName;


    m_image = cv::imread(path, cv::IMREAD_GRAYSCALE);
    m_colorImage = cv::imread(path, cv::IMREAD_COLOR);


    if (m_image.empty()) {
        std::cout << "\n\n\nImage not found!" << std::endl;
        return false;
    }
    if (m_colorImage.empty()) {
        return false;
    }

    std::cout << "Image shape: " << m_image.cols << "x" << m_image.rows << std::endl;
    /*cv::Point minLoc, maxLoc;
    float minVal, maxVal;
    cv::minMaxLoc(m_image, &minVal, &maxVal, &minLoc, &maxLoc);
    std::cout << "Max pixel value: " << maxVal << ", Min pixel value: " << minVal << std::endl;*/
    return true;
}

bool WeldCircleInspection::CalculateCircleBead() {
    std::cout << "\nWelding Bead Area : " << m_nWeldingBeadPixelCnt << " pixels\n";

    /* ROI --- RIGHT, BOTTOM, LEFT, TOP 계산*/
    // 각도, 좌표 (x,y )
    cv::Point3i pt3 = { 0,0,0 };
    int nAngleCnt = 4;
    std::vector<cv::Point3i> vecAngleQuater(nAngleCnt, pt3 );

    for (int i = 0; i < vecAngleQuater.size(); i++) {
        int nIdx= i * (m_vecPolarInner.size() / nAngleCnt);
        int nDegree = i * 90;

        int x = m_vecCartesianOuter[nIdx].x;
        int y = m_vecCartesianOuter[nIdx].y;

        vecAngleQuater[i] = { nDegree ,x, y };
    }

    for (auto i : vecAngleQuater) {
        int nDegree = i.x;
        int x = i.y;
        int y = i.z;

        int nHalf = 20;
        cv::Rect rect(x- nHalf, y- nHalf, nHalf*2, nHalf*2);
        cv::rectangle(m_colorImage, rect, cv::Scalar(0, 255, 0), THICKNESS);
    }
    


    return true;
}
