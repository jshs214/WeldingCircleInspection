#include "WeldCircleInspection.h"

int developMode;

WeldCircleInspection::WeldCircleInspection(eType type) {
    m_EnumType = type;

    // hardcoding
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
    break;
    }
    
    llWeldCircleDetectTime = 0;
    developMode = 0;     // 0 : false   ||  value : true
    
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
bool WeldCircleInspection::LargeCircleDetect(cv::Mat inputMat, cv::Mat& outputMat, double minDist,
    double param1, double param2,
    int minRadius, int maxRadius) {

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(inputMat, circles,
        cv::HOUGH_GRADIENT, 1, minDist,
        param1, param2,
        minRadius, maxRadius);

    if (circles.empty()) {
        std::cout << "==================================\n";
        std::cout << "Hough circle detect FAILED\n";
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
    }

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
bool WeldCircleInspection::WeldCircleDetect() {
    // [ 0. Read]
    TimeStart();
    std::string path = m_filePath + "90.bmp";
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
        SaveBmp(preProcImg, "1) pre-Processing Image");
    TimeEnd("\n\n 1) Processing Time : ");

    // [ 2. Hough Circle Transform (Large Circle detect) ]
    TimeStart();
    double param1 = 50, param2 = 30, minDist = 3000;        // magicNum
    int minRadius = 700, maxRadius = 800;
    if (!LargeCircleDetect(preProcImg, m_colorImage, minDist, param1, param2, minRadius, maxRadius))
        return false;
    if (developMode) {
        SaveBmp(m_colorImage, "2) [houghCl] Image");
        cv::imshow("Hough Circles", resizeShow(m_colorImage));
        cv::waitKey(0);
    }
    TimeEnd("\n\n 2) Large Circle Inspection Time : ");

    
    // [ 3. Find small Circle ]
    TimeStart();
    if (!SmallCircleDetect(preProcImg))
        return false;
    TimeEnd("\n\n 3) Small Circle Inspection Time : ");

    // check Total Time
    std::cout << "\n\n [Total Time] " << llWeldCircleDetectTime << "ms\n";
    std::cout << "\n\n [Weld Bead Width] min(" << m_fMinWeldWidth << ") ~ max(" << m_fMaxWeldWidth << ")\n";
    cv::imshow("Result", resizeShow(m_colorImage));
    cv::waitKey(0);

    return true;
}
/* Least-Squares */
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

/* @return : slope_X, intercept_X*/
std::pair<float, float> WeldCircleInspection::Ransac(std::vector<cv::Point>& points, int maxIterations,
    float threshold, int& inlierCount) {
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

        step 1 : cartesian To polar (circle image unwrapped)
        step 2 : unwrapped image To binary image
        step 3 : Width-axis Edge inspection
        step 4 : make small Circle
*/
bool WeldCircleInspection::SmallCircleDetect(cv::Mat& _image) {

    cv::Mat image = _image;
    const int nAngle = 360;
    const int r = m_nCircumscribedRadius;
    const int cx = m_cartesianCenter.x;
    const int cy = m_cartesianCenter.y;

    // Unwrapp image Size : (circle radius * 360)
    int imgWidth = r;
    int imgHeight = nAngle;

    cv::Mat unwrapimage = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC1);

    for (int height = 0; height < imgHeight; ++height) {
        double radian = height * M_PI / 180.0;  // degree To radian

        for (int width = 0; width < imgWidth; ++width) {
            // 1-1) cartesian To polar
            int x = (int)(width * cos(radian)) + cx;
            int y = (int)(width * sin(radian)) + cy;

            // 1-2) Make Unwrapped image
            if (x >= 0 && x < m_image.cols && y >= 0 && y < m_image.rows) {
                int origValue = (int)image.at<uchar>(y, x);
                unwrapimage.at<uchar>(height, width) = static_cast<uchar>(origValue);
            }
        }
    }

    //2) unwrapimage To binary
    cv::Mat sobelX, sobelY, magnitudeImage;
    cv::Sobel(unwrapimage, sobelX, CV_8UC1, 1, 0, 3);


    cv::Mat binary;
    int nThreshold = 40;
    int nBinaryMax = 255;
    cv::threshold(sobelX, binary, nThreshold, nBinaryMax, cv::THRESH_BINARY);                // magicNum

    // 3) Width-axis Edge inspection
    std::vector<cv::Point>edgePoints;

    for (int y = 0; y < unwrapimage.rows; y++) {
        int maxEdgeX = -1;
        int maxEdgeValue = -1;

        //----- Range check-----
        for (int x = unwrapimage.cols / 2; x < unwrapimage.cols; x++) {
            int edgeValue = binary.at<uchar>(y, x);
            if (edgeValue == nBinaryMax) {
                maxEdgeValue = edgeValue;
                maxEdgeX = x;
                break;  //처음 등장하는 Edge 검출
            }
        }
        if (maxEdgeX != -1) {
            edgePoints.push_back(cv::Point(maxEdgeX, y));
        }
    }

    // 4) make small Circle
    // ransac parameter
    int maxIterations = 100000;         // iterator
    float ransacThreshold = 0.05f;      // Inlier Threshold
    int inlierCount = 0;
    // 4-1) ransac
    std::pair<float, float> fX = Ransac(edgePoints, maxIterations, ransacThreshold, inlierCount);

    float slopeX = fX.first;
    float interceptX = fX.second;
    // Inscribed Circel info
    for (int i = 0; i < nAngle; i++) {
        // y = mX + b
        float fRadius = i * slopeX + interceptX;
        m_vInscribed.push_back({ i, fRadius });

        float fWeldBeadWidth = (float)m_nCircumscribedRadius - fRadius;
        m_vWeldBeadWidth.push_back({ i, fWeldBeadWidth });

        m_fMinWeldWidth = std::min(m_fMinWeldWidth, fWeldBeadWidth);
        m_fMaxWeldWidth = std::max(m_fMaxWeldWidth, fWeldBeadWidth);
    }
    
    // 4-2) draw
    cv::Mat resultImage;
    cv::cvtColor(unwrapimage, resultImage, cv::COLOR_GRAY2BGR);
    cv::line(resultImage, cv::Point(interceptX, 0), cv::Point(interceptX, unwrapimage.rows), cv::Scalar(0, 0, 255), 1);

    // polar To cartesian Draw
    for (const auto& polar : m_vInscribed){
        float radian = polar.first;
        float r= polar.second;
        int x = (int)(r * cos(radian)) + m_cartesianCenter.x;
        int y = (int)(r * sin(radian)) + m_cartesianCenter.y;

        cv::circle(m_colorImage, cv::Point(x , y ), 2, cv::Scalar(0, 0, 255), 2);
    }

    if (developMode) {
        SaveBmp(unwrapimage, "3.1) Unwrapped image");
        SaveBmp(sobelX, "3.2) sobel");
        SaveBmp(binary, "3.3) binary");
        SaveBmp(resultImage, "3.4) Least-Squares Img");
        SaveBmp(m_colorImage, "Welding Circle Inspection");
        cv::imshow("circle Inspection", resizeShow(m_colorImage));
        cv::waitKey(0);
    }
    else {
        SaveBmp(resultImage, "3-1) Least-Squares Img");
        SaveBmp(m_colorImage, "3-2) Welding Circle Inspection");
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
    m_outputPath = m_filePath.substr(0, m_filePath.size() - 1) + "Test\\";

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