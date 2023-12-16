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
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    Eigen::Vector3f p(x, y, 0);
    Eigen::Vector3f pa = _v[0] - p, pb = _v[1] - p, pc = _v[2] - p;

    Eigen::Vector3f crossPaPb, crossPbPc, crossPcPa;
    crossPaPb = pa.cross(pb);
    crossPbPc = pb.cross(pc);
    crossPcPa = pc.cross(pa);

    float temp1 = crossPaPb.dot(crossPbPc);
    float temp2 = crossPbPc.dot(crossPcPa);
    float temp3 = crossPcPa.dot(crossPaPb);

    // 判断是否同号，同号则在三角形内
    return (temp1 > 0 && temp2 > 0 && temp3 > 0) || (temp1 < 0 && temp2 < 0 && temp3 < 0);
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
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
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
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    float xMin = width, xMax = 0, yMin = height, yMax = 0;
   
    for (int i = 0; i < 3; i++)
    {
        float curX = v[i].x();
        float curY = v[i].y();

        if (xMin > curX)
            xMin = curX;
        if (xMax < curX)
            xMax = curX;
        if (yMin > curY)
            yMin = curY;
        if (yMax < curY)
            yMax = curY;
    }

    for (int x = (int)xMin; x <= xMax; ++x)
    {
        for (int y = (int)yMin; y <= yMax; ++y)
        {
            // 是否使用MSAA反走样
            int openMSAA = true;
            if (openMSAA)
            {
                float depth = 0x3f3f3f3f;
                int insideCount = 0;

                for (int i = 0; i < 4; i++)
                {
                    float newX = x + rst::MSAA_step[i][0];
                    float newY = y + rst::MSAA_step[i][1];

                    // (x,y)点是否在三角形内
                    if (insideTriangle(newX, newY, t.v))
                    {
                        // 已有代码，计算插值深度值
                        auto tmp = computeBarycentric2D(newX, newY, t.v);
                        float alpha, beta, gamma;
                        std::tie(alpha, beta, gamma) = tmp;
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        int curIndex = y * width * 4 + x + i;
                        if (z_interpolated < depth_buf_MSAA[curIndex])
                        {
                            // 更新深度值
                            depth_buf_MSAA[curIndex] = z_interpolated;
                            frame_buf_MSAA[curIndex] = t.getColor();
                        }
                        depth = std::min(depth, z_interpolated);
                        insideCount++;
                    }
                }
                if (insideCount > 0)
                {
                    // 更新深度值
                    int indexOrigin = y * width + x;
                    depth_buf[indexOrigin] = depth;

                    // 设置像素点的颜色
                    int curIndex = y * width * 4 + x;
                    Eigen::Vector3f color = (frame_buf_MSAA[curIndex] + frame_buf_MSAA[curIndex + 1] + frame_buf_MSAA[curIndex + 2] + frame_buf_MSAA[curIndex + 3]) / 4;
                    set_pixel(Eigen::Vector3f(x, y, depth), color);
                }
            }
            else
            {
                int index = y * width + x;
                // (x,y)点是否在三角形内
                if (insideTriangle(x, y, t.v))
                {
                    // 已有代码，计算插值深度值
                    auto tmp = computeBarycentric2D(x, y, t.v);
                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = tmp;
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // 深度比缓冲区中更大，直接跳过
                    if (z_interpolated > depth_buf[index])
                        continue;

                    // 更新深度值
                    depth_buf[index] = z_interpolated;

                    // 设置像素点的颜色
                    set_pixel(Eigen::Vector3f(x, y, z_interpolated), t.getColor());
                }
            }
        }
    }
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
        std::fill(frame_buf_MSAA.begin(), frame_buf_MSAA.end(), Eigen::Vector3f{ 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_MSAA.begin(), depth_buf_MSAA.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    frame_buf_MSAA.resize(w * h * 4);
    depth_buf.resize(w * h);
    depth_buf_MSAA.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on