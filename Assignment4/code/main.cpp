#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

bool AA = false;
void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 5)
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
    if (control_points.size() == 2) {
        return cv::Point2f( control_points[0].x * (1-t) + control_points[1].x * t,
                            control_points[0].y * (1-t) + control_points[1].y * t);
    }
    std::vector<cv::Point2f> control_points_temp;
    for (int i = 0; i < control_points.size() - 1; i++) {
        control_points_temp.push_back(cv::Point2f( control_points[i].x * (1-t) + control_points[i+1].x * t,
                                                      control_points[i].y * (1-t) + control_points[i+1].y * t));
    }

    return recursive_bezier(control_points_temp, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    if (AA) {
        for (float t = 0.0; t <= 1.0; t += 0.0001) {
            auto [x, y] = recursive_bezier(control_points, t);
            float pixel_center_x[2] = {std::round(x) - 0.5f, std::round(x) + 0.5f};
            float pixel_center_y[2] = {std::round(y) - 0.5f, std::round(y) + 0.5f};
            float distance_square[2][2];

            for (int xi = 0; xi < 2; ++xi) {
                for (int yi = 0; yi < 2; ++yi) {
                    distance_square[xi][yi] = std::pow(pixel_center_x[xi] - x, 2) + std::pow(pixel_center_y[yi] - y, 2);
                    int color = 255 * (std::sqrt(2) - std::sqrt(distance_square[xi][yi])/std::sqrt(2));

                    window.at<cv::Vec3b>(pixel_center_y[yi], pixel_center_x[xi])[1] =
                        std::max((int)window.at<cv::Vec3b>(pixel_center_y[yi], pixel_center_x[xi])[1], color);
                }
            }
        }
    } else {
        for (float t = 0.0; t <= 1.0; t += 0.0001) {
            auto point = recursive_bezier(control_points, t);
            window.at<cv::Vec3b>(point.y, point.x)[1] = 255;   // 注意这里先y再x
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

        if (control_points.size() == 5)
        {
            // naive_bezier(control_points, window);
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
