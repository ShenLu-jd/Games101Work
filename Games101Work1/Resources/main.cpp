#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotation_z;
    float cosX = cos(rotation_angle / 180.0 * MY_PI);
    float sinX = sin(rotation_angle / 180.0 * MY_PI);
    rotation_z <<
        cosX, -sinX, 0, 0,
        sinX, cosX, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    model = rotation_z;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // 透视投影矩阵需要 l,r,t,b的信息，需要根据FOV和aspect来计算得出
    // tan(eye_fov / 2) = t / zNear 可以求出 t
    // aspect_ratio = w / h = r / t; 由上式求出t, h = 2*t代入，求出w， 这样就能得到 所有的信息
    
    float l, r, t, b, n, f;

    n = zNear;
    f = zFar;
    t = atan(eye_fov / 180.0 * MY_PI / 2) * (-n); // 使用右手坐标系，所以为-n
    float w = aspect_ratio * t * 2;
    r = w / 2;
    l = -r;
    b = -t;


    std::cout << "l=" << l << ", r=" << r << ", t=" << t << ", b=" << b << ", n=" << zNear << ", f=" << zFar << "\n";

    Eigen::Matrix4f ortho, persp;

    persp <<
        n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    ortho <<
        2 / (r - l), 0, 0, -(r + l) / (r - l),
        0, 2 / (t - b), 0, -(t + b) / (t - b),
        0, 0, 2 / (n - f), -(n + f) / (n - f),
        0, 0, 0, 1;


    //Eigen::Matrix4f p;
    //p <<
    //    2 * n / (r - l), 0, (l + r) / (l - r), 0,
    //    0, (2 * n) / (t - b), (b + t) / (b - t), 0,
    //    0, 0, (f + n) / (n - f), 2 * f * n / (f - n),
    //    0, 0, 1, 0;

    projection = ortho * persp;

    return projection;
}

// 绕任意轴旋转
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotation_z, N, R;
    Eigen::Vector4f n;
    Eigen::RowVector4f nt;
    n << axis.x(), axis.y(), axis.z(), 0;
    nt<< axis.x(), axis.y(), axis.z(), 0;

    N <<
        0, -n.z(), n.y(), 0,
        n.z(), 0, -n.x(), 0,
        -n.y(), n.x(), 0, 0,
        0, 0, 0, 1;

    float a = angle / 180.0 * MY_PI;

    R = cos(a) * I + (1 - cos(a)) * n * nt + sin(a) * N;
    R(3,3) = 1;

    return R;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    Vector3f axis;
    float rangle = 0;

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

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    std::cout << "请分别输入x y z轴\n" << std::endl;
    std::cin >> axis.x() >> axis.y() >> axis.z();

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

        r.set_model(get_rotation(axis, rangle) * get_model_matrix(angle));
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
        else if (key == 'w') {
            rangle += 10;
        }
        else if (key == 's') {
            rangle -= 10;
        }
    }

    return 0;
}
