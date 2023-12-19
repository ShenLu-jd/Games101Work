#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
int clickNum = 4;
void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < clickNum)
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
    if (control_points.size() == 1)
    {
        return control_points[0];
    }

    std::vector<cv::Point2f> control_points_recu;

    for (int i = 1; i < control_points.size(); i++)
    {
        cv::Point2f p0 = control_points[i - 1];
        cv::Point2f p1 = control_points[i];

        control_points_recu.push_back(p0 + t * (p1 - p0));
    }

    cv::Point2f point = recursive_bezier(control_points_recu, t);

    return point;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    for (float t = 0.0f; t <= 1.0f; t += 0.001f)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        // 反走样
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 && j == 0) continue;
                if (point.x + i >= 700 || point.y + j >= 700 || point.x + i < 0 || point.y + j < 0) continue;
                // point（x,y）与周围的8个格子的距离，最近为0，最大为斜对角3*sqrt(2)/2，也就是3/sqrt(2)
                // 0 <= d <= 3/sqrt(2)
                // ratio 代表颜色值，范围为[0,1]*255, 也就是[0,255]
                // ratio关于d的函数就是 ratio = 1 - sqrt(2)/3 * d  
                // 距离d越小，那么颜色值越大，距离d越大，颜色值越小
                // 0 <= ratioColor <= 255
                // ratioColor = 1 - sqrt(2)/3 * d
                float x = (int)point.x + i * 0.5f;
                float y = (int)point.y + j * 0.5f;
                float d = sqrt(pow(point.x - x, 2) + pow(point.y - y, 2));
                int ratio = (1 - sqrt(2)/3 * d) * 255;
                window.at<cv::Vec3b>(point.y + j, point.x + i)[1] = fmax(ratio, window.at<cv::Vec3b>(point.y + j, point.x + i)[1]);            
            }
        }
    }
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

        if (control_points.size() == clickNum)
        {
            //naive_bezier(control_points, window);
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
