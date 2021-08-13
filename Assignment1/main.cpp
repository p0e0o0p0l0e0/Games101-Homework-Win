#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Matrix4f get_rotation(Vector3f axis, float angle)
{
    Matrix3f I = Matrix3f::Identity();
    Matrix3f n = Matrix3f::Identity();

    float rad = angle / 180.0 * MY_PI;

    float x = axis.x();
    float y = axis.y();
    float z = axis.z();
    n << 0, -z, y,
        z, 0, -x,
        -y, x, 0;
    Matrix3f R = cos(rad) * I + (1 - cos(rad)) * axis * axis.transpose() + sin(rad) * n;
    Matrix4f result;
    result << R.row(0), 0,
        R.row(1), 0,
        R.row(2), 0,
        0, 0, 0, 1;
    return result;
}

Matrix4f get_view_matrix(Vector3f eye_pos)
{
    Matrix4f view = Matrix4f::Identity();

    Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Matrix4f get_model_matrix(float rotation_angle)
{
    Matrix4f model = Matrix4f::Identity();

    Matrix4f rotate;
    float rad = rotation_angle / 180.0 * MY_PI;
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);
    rotate << cosA, -sinA, 0, 0, sinA, cosA, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    model = rotate * model;

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    return model;
}

Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)
{
    // Students will implement this function

    Matrix4f projection = Matrix4f::Identity();

    float n = -zNear * 1.0;
    float f = -zFar * 1.0;
    float halfFov = eye_fov / 180.0 / 2 * MY_PI;
    float t = std::tan(halfFov) * zNear;
    float r = t * aspect_ratio;

    // orthographic projection
    Matrix4f p2o;
    p2o << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    // translate to center (0, 0, 0)
    Matrix4f translate;
    translate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, -(n + f) / 2, 0,
        0, 0, 0, 1;

    // scale to canonical cube (-1, 1)^3 
    Matrix4f scale;
    scale << 1 / r, 0, 0, 0,
        0, 1 / t, 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;

    Matrix4f Mortho = scale * translate;
    Matrix4f proj = Mortho * p2o;
    projection = proj * projection;

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

    std::vector<Vector3i> ind{ {0, 1, 2} };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle)); // »ù´¡
        r.set_model(get_rotation(Vector3f(1, 0, 0), angle));// Ìá¸ß
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        std::cout << "angle: " << angle << std::endl;
    }

    return 0;
}