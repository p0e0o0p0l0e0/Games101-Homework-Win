// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // 作业3框架里的内容
    /*Vector3f v[3];
    for (int i = 0; i < 3; i++)
        v[i] = { _v[i].x(),_v[i].y(), 1.0 };
    Vector3f f0, f1, f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x, y, 1.);
    if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
        return true;
    return false;*/

    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f point(x, y, _v[0].z());
    Vector3f vec0 = _v[0] - _v[1];
    Vector3f vec1 = _v[1] - _v[2];
    Vector3f vec2 = _v[2] - _v[0];

    Vector3f _vec0 = point - _v[1];
    Vector3f _vec1 = point - _v[2];
    Vector3f _vec2 = point - _v[0];

    float c1, c2, c3;
    c1 = _vec0.cross(vec0).z();
    c2 = _vec1.cross(vec1).z();
    c3 = _vec2.cross(vec2).z();

    // 不管传入的顶点是逆时针还是顺时针都可以判断是否在三角形内
    if ((c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 < 0 && c2 < 0 && c3 < 0))
    {
        return true;
    }
    return false;
}

static float multiSample(int x, int y, const Vector3f* _v)
{
    std::vector<Vector3f> samplingPoints
    {
         Vector3f(x + 0.25f, y + 0.25f, _v[0].z()),
         Vector3f(x + 0.75f, y + 0.25f, _v[0].z()),
         Vector3f(x + 0.25f, y + 0.75f, _v[0].z()),
         Vector3f(x + 0.75f, y + 0.75f, _v[0].z()),
    };
    float percentage = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        Vector3f point = samplingPoints[i];
        if (insideTriangle(point.x(), point.y(), _v))
        {
            percentage += 0.25f;
        }
    }
    return percentage;
}

static Vector3f getMSAAColor(int x, int y, const Triangle& t)
{
    return t.getColor() * multiSample(x, y, t.v);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 - f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            // TODO : 这里为什么要写三次?
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
    rasterize_ssaa_triangle();
}

void rst::rasterizer::rasterize_ssaa_triangle()
{
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            set_pixel(x, y, get_color4(x, y));
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    int minX = std::max(0, (int)std::floor(std::min(v[0].x(), std::min(v[1].x(), v[2].x()))));
    int maxX = std::min(width - 1, (int)std::ceil(std::max(v[0].x(), std::max(v[1].x(), v[2].x()))));
    int minY = std::max(0, (int)std::floor(std::min(v[0].y(), std::min(v[1].y(), v[2].y()))));
    int maxY = std::min(height - 1, (int)std::ceil(std::max(v[0].y(), std::max(v[1].y(), v[2].y()))));

    /*
    for (int x = minX; x <= maxX; x++)
    {
        for (int y = minY; y <= maxY; y++)
        {
            if (insideTriangle(x + 0.5f, y + 0.5f, t.v))
            {
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                auto ind = get_index(x, y);
                if (std::isinf(depth_buf[ind]) || depth_buf[ind] < z_interpolated)
                {
                    depth_buf[ind] = z_interpolated;
                    //set_pixel(x, y, t.getColor()); // 未做AA
                    set_pixel(x, y, getMSAAColor(x, y, t)); // MSAA
                }
            }
        }
    }*/
    

    // SSAA
    for (int x = minX; x <= maxX; x++)
    {
        for (int y = minY; y <= maxY; y++)
        {
            std::vector<Vector3f> samplingPoints
            {
                Vector3f(x + 0.25f, y + 0.25f, t.v[0].z()),
                Vector3f(x + 0.75f, y + 0.25f, t.v[0].z()),
                Vector3f(x + 0.75f, y + 0.75f, t.v[0].z()),
                Vector3f(x + 0.25f, y + 0.75f, t.v[0].z()),
            };
            for (int i = 0; i < 4; i++)
            {
                if (insideTriangle(samplingPoints[i].x(), samplingPoints[i].y(), t.v))
                {
                    auto [alpha, beta, gamma] = computeBarycentric2D(samplingPoints[i].x(), samplingPoints[i].y(), t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    auto ind = get_index4(x, y, i);
                    if (std::isinf(depth_buf4[ind]) || depth_buf4[ind] < z_interpolated)
                    {
                        depth_buf4[ind] = z_interpolated;
                        set_pixel4(x, y, t.getColor(), i);
                    }
                }
            }
        }
    }

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf4.begin(), frame_buf4.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf4.begin(), depth_buf4.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf4.resize(w * h * 4);
    depth_buf4.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(int x, int y, const Eigen::Vector3f& color)
{
    auto ind = get_index(x, y);
    frame_buf[ind] = color;
}

// 即使采样点是2*2，index可以直接width*4扩展存储，无需width*2，height*2这样计算。
int rst::rasterizer::get_index4(int x, int y, int index)
{
    int ind = 0;
    ind = ((height - 1 - y) * width + x) * 4 + index;
    return ind;
}

void rst::rasterizer::set_pixel4(int x, int y, const Eigen::Vector3f& color, int index)
{
    auto ind = get_index4(x, y, index);
    frame_buf4[ind] = color;
}

Vector3f rst::rasterizer::get_color4(int x, int y)
{
    Vector3f finalColor = Vector3f(0, 0, 0);
    for (int i = 0; i < 4; i++)
    {
        auto ind = get_index4(x, y, i);
        finalColor += frame_buf4[ind];
    }
    return finalColor / 4;
}

float rst::rasterizer::get_depth4(int x, int y)
{
    for (int i = 0; i < 4; i++)
    {
        auto ind = get_index4(x, y, i);
        return depth_buf4[ind];
    }
}

// clang-format on