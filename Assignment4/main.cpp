#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
std::vector<cv::Point2f> newPoints;
int leftPointSize;
int minSize = 4;

/*
// 加权掩模相关
class PixelValue
{
public:
    int pixelX = 0;
    int pixelY = 0;
    float originalX = 0.f;
    float originalY = 0.f;
    std::vector<int> pixelRecords;

    void AddRecord(int pos)
    {
        pixelRecords[pos] = 1;
    }

    void SetPixel(int x, int y)
    {
        pixelX = x;
        pixelY = y;
    }

    float GetValue()
    {
        float f1 = 1.f / 16;
        float f2 = 2.f / 16;
        float f4 = 4.f / 16;
        float val = (pixelRecords[0] + pixelRecords[2] + pixelRecords[6] + pixelRecords[8]) * f1
            + (pixelRecords[1] + pixelRecords[3] + pixelRecords[5] + pixelRecords[7]) * f2
            + pixelRecords[4] * f4;
        return val;
    }

    PixelValue(float x, float y)
    {
        pixelX = x;
        pixelY = y;
        originalX = x;
        originalY = y;
        pixelRecords = std::vector<int>(9);
    }
};

std::vector<PixelValue> subPixelValues;
PixelValue* GetPixelValue(int x, int y)
{
    PixelValue *newPV = NULL;
    for (int i = subPixelValues.size() - 1; i >= 0; i--)
    {
        PixelValue pv = subPixelValues[i];
        if (pv.pixelX == x && pv.pixelY == y)
        {
            newPV = &subPixelValues[i];
            break;
        }
    }
    if (newPV == NULL)
    {
        newPV = new PixelValue(x, y);
        subPixelValues.push_back(*newPV);
        newPV = &subPixelValues[subPixelValues.size() - 1];
    }
    return newPV;
}

int GetPixelPos(float dx, float dy)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (dx >= i / 3.f && dx < (i + 1) / 3.f && dy >= j / 3.f && dy < (j + 1) / 3.f)
            {
                return 3 * j + i;
            }
        }
    }
    return -1;
}

void AddPixel(float x, float y)
{
    int rx = x;
    int ry = y;
    PixelValue *pv = GetPixelValue(rx, ry);
    int pos = GetPixelPos(x - rx, y - ry);
    if (pos > 0)
    {
        pv->AddRecord(pos);
    }
}

// 平均颜色值相关
std::vector<AssistPixel> assistPixels;
class AssistPixel
{
public:
    int x;
    int y;
    int count;
    float color;

    AssistPixel(int _x, int _y, float _color)
    {
        x = _x;
        y = _y;
        color = _color;
        count = 1;
    }
};

void AddAssistPixel(int x, int y, float color)
{
    for (int i = assistPixels.size() - 1; i >= 0; i--)
    {
        if (assistPixels[i].x == x && assistPixels[i].y == y)
        {
            assistPixels[i].color += color;
            assistPixels[i].count++;
            return;
        }
    }
    AssistPixel* ap = new AssistPixel(x, y, color);
    assistPixels.push_back(*ap);
}

void setAssistPixelColor(cv::Mat& window)
{
    for (int i = 0; i < assistPixels.size(); i++)
    {
        window.at<cv::Vec3b>(assistPixels[i].y, assistPixels[i].x)[1] = clamp(0.f, 255.f, assistPixels[i].color / assistPixels[i].count);
    }
}

void setColorByWeightSubPixels(cv::Mat& window)
{
    for (int i = 0; i < subPixelValues.size() - 1; i++)
    {
        float weight = clamp(0, 1, subPixelValues[i].GetValue());
        window.at<cv::Vec3b>(subPixelValues[i].pixelY, subPixelValues[i].pixelX)[1] = weight * 255.f;
    }
}
*/

float clamp(float raw, float low, float high)
{
    if (raw < low)
        return low;
    if (raw > high)
        return high;
    return raw;
}

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < minSize)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (leftPointSize == 1)
    {
        return control_points[0];
    }
    else
    {
        for (int i = 0; i < leftPointSize - 1; i++)
        {
            newPoints[i] = (1 - t) * control_points[i] + t * control_points[i + 1];
        }
        leftPointSize--;
        return recursive_bezier(newPoints, t);
    }
}

float currColorValue(float x, float y, float centerx, float centery)
{
    // 用面积计算当前像素的初始色值，3为调节系数
    //return std::abs((x - centerx) * (y - centery)) * 255 * 3;
    
    // 用1 - 距离除以根号2作为颜色比例，1.25作为调节系数
    return (1 - std::sqrt(std::pow((x - centerx), 2) + std::pow((y - centery), 2)) / std::sqrt(2)) * 255 * 1.25;
}

void finalColorValue(int x, int y, float colorValue, cv::Mat& window)
{
    int offset1 = 20, offset2 = 40, offset3 = 60;

    // 像素点颜色平均
    // AddAssistPixel(x, y - offset1, colorValue);

    // 关键点周围的像素值取最大（上移offset2像素)（当步长更短时，这个效果最好）
    window.at<cv::Vec3b>(y - offset2, x)[1] = clamp(std::max(colorValue, (float)window.at<cv::Vec3b>(y - offset2, x)[1]), 0.f, 255.f);

    // 关键点周围的像素值叠加
    //window.at<cv::Vec3b>(y - offset3, x)[1] = clamp((colorValue + (float)window.at<cv::Vec3b>(y - offset3, x)[1]), 0.f, 255.f);
}

void setColor(cv::Point2f point, cv::Mat &window)
{
    cv::Point2i nearestPoint = cv::Point2i(std::round(point.x), std::round(point.y));
    cv::Point2f centerPoint = cv::Point2f(nearestPoint.x + 0.5, nearestPoint.y + 0.5);
    cv::Point2i basePoint = cv::Point2i(point.x, point.y);

    // 右上
    //float colorValue1 = currColorValue(point.x, point.y, centerPoint.x - 1, centerPoint.y - 1); // 用面积计算
    float colorValue1 = currColorValue(point.x, point.y, centerPoint.x, centerPoint.y); // 用距离计算
    // 左下
    //float colorValue2 = currColorValue(point.x, point.y, centerPoint.x, centerPoint.y); // 用面积计算
    float colorValue2 = currColorValue(point.x, point.y, centerPoint.x - 1, centerPoint.y - 1); // 用距离计算
    // 左上
    //float colorValue3 = currColorValue(point.x, point.y, centerPoint.x, centerPoint.y - 1); // 用面积计算
    float colorValue3 = currColorValue(point.x, point.y, centerPoint.x - 1, centerPoint.y); // 用距离计算
    // 右下
    //float colorValue4 = currColorValue(point.x, point.y, centerPoint.x - 1, centerPoint.y); // 用面积计算
    float colorValue4 = currColorValue(point.x, point.y, centerPoint.x, centerPoint.y - 1); // 用距离计算

    // 直接上色，叠加或者取最大或平均
    finalColorValue(nearestPoint.x, nearestPoint.y, colorValue1, window);
    finalColorValue(nearestPoint.x - 1, nearestPoint.y - 1, colorValue2, window);
    finalColorValue(nearestPoint.x - 1, nearestPoint.y, colorValue3, window);
    finalColorValue(nearestPoint.x, nearestPoint.y - 1, colorValue4, window);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    newPoints = std::vector<cv::Point2f>(control_points.size() - 1);
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        leftPointSize = control_points.size();
        cv::Point2f point = recursive_bezier(control_points, t); 

        setColor(point, window); // 根据面积或距离计算周围像素颜色进行反走样

		//AddPixel(point.x, point.y); // 加权掩模逻辑
    }
    // setColorByWeightSubPixels(window); // 加权掩膜逻辑 像素拆成9宫格进行加权掩模反走样，需要对周围像素格处理，否则颜色太浅
    // setAssistPixelColor(window); // 颜色值取平均
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == minSize)
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
