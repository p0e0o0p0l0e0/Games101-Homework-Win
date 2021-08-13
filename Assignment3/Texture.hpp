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
        auto u_img = std::clamp(u, 0.f, 1.f) * width;
        auto v_img = std::clamp((1 - v), 0.f, 1.f) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = std::clamp(u, 0.f, 1.f) * width;
        auto v_img = std::clamp((1 - v), 0.f, 1.f) * height;

        int iu_img = std::round(u_img);
        int iv_img = std::round(v_img);

        Eigen::Vector2i samplingPoints[4];
        samplingPoints[0] = { iu_img - 1, iv_img };
        samplingPoints[1] = { iu_img - 1, iv_img - 1};
        samplingPoints[2] = { iu_img, iv_img };
        samplingPoints[3] = { iu_img, iv_img - 1 };
        if (iu_img == 0)
        {
            for (int i = 0; i < 4; i++)
            {
                samplingPoints[i].x()++;
            }
        }
        if (iv_img == 0)
        {
            for (int i = 0; i < 4; i++)
            {
                samplingPoints[i].y()++;
            }
        }

        // 0 --------- 2
        // |           |
        // W1-- (uv) --W2
        // |           |
        // 1 --------- 3

        Eigen::Vector3f samplingColor[4];
        for (int i = 0; i < 4; i++)
        {
            cv::Vec3b _c = image_data.at<cv::Vec3b>(samplingPoints[i].y(), samplingPoints[i].x());
            samplingColor[i] = Eigen::Vector3f(_c[0], _c[1], _c[2]);
        }
        // 注意：离得越近的纹素，比重更大，但distance = samplingPoints[0].y() + 0.5f - v_img更小，所以需要用1 - distance来算比重。
        float v1 = samplingPoints[0].y() + 0.5f - v_img;
        Eigen::Vector3f W1 = (1 - v1) * samplingColor[0] + v1 * samplingColor[1];
        Eigen::Vector3f W2 = (1 - v1) * samplingColor[2] + v1 * samplingColor[3];
        
        float u1 = u_img - (samplingPoints[1].x() + 0.5f);
        Eigen::Vector3f color = (1 - u1) * W1 + u1 * W2;;

        return color;
    }

};
#endif //RASTERIZER_TEXTURE_H
