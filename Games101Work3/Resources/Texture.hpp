//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u = std::clamp(u, 0.0f, 1.0f);
        v = std::clamp(v, 0.0f, 1.0f);

        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        float u_floor = std::floor(u_img);
        float v_floor = std::floor(v_img);

        //float u0 = std::max(0.0f, u_floor - 0.5f);
        //float u1 = std::min((float)width - 1, u_floor + 0.5f);

        //float v0 = std::max(0.0f, v_floor - 0.5f);
        //float v1 = std::min((float)height-1, v_floor + 0.5f);

        float u0 = u_floor;
        float u1 = std::ceil(u_img);

        float v0 = v_floor;
        float v1 = std::ceil(v_img);

        auto color00 = image_data.at<cv::Vec3b>(v0, u0);
        auto color01 = image_data.at<cv::Vec3b>(v1, u0);
        auto color10 = image_data.at<cv::Vec3b>(v0, u1);
        auto color11 = image_data.at<cv::Vec3b>(v1, u1);

        //float s = (u_img - u0) / (u1 - u0);
        //float t = (v_img - v0) / (v1 - v0);

        float s = (u_img - u0);
        float t = (v_img - v0);
        
        auto u0_color = color00 + s * (color10 - color00);
        auto u1_color = color01 + s * (color11 - color01);

        auto color = u0_color + t * (u1_color - u0_color);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
