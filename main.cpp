#include <fstream>
#include<stdio.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<algorithm>
#include<opencv2/opencv.hpp>

using namespace std;

const float PI = 4 * atan(1);

struct _Plane {
    float A;
    float B;
    float C;
    float D;
};

struct _Line
{
    float a;
    float b;
    float c;
};

struct _Vec_Point_Pair
{
    //! 位于激光平面的坐标vec
    std::vector<cv::Point3f> p1;
    //! 位于传感器(图像)平面的坐标vec
    std::vector<cv::Point3f> p2;
};

//! theta_1[已知] 激光平面与光轴夹角
const float theta_1 = 45.0 / 180 * PI;

//! theta_2[已知] 光轴与传感器平面夹角
const float theta_2 = 60.0 / 180 * PI;

//! theta_3[已知] 激光平面与传感器平面夹角
const float theta_3 = 75.0 / 180 * PI;

//! theta_4[未知] 激光平面与镜头平面夹角
float theta_4;

//! theta_9[未知] 坐标系旋转角度
float theta_9 = PI / 2 - theta_3;


//! a[已知] 为激光平面至镜头光心的距离
//! b[已知] 为镜头光心至传感器的距离
const float distance_a = -50.0, distance_b = -35.0;

//! p1为激光平面
const _Plane p1 = { 0,1.0,0,0 };


//! p2为成像平面（传感器平面）
//! 假设p2平面是由方向向量为(0,0,-1) 对应 (i,j,k) 的平面绕x轴逆时针旋转PI/2 - theta_3个弧度得到的平面，那么宣传后的法向量为
//!														(0,j*cos(theta_3)−k*sin(theta_3),j*sin(theta_3)+k*cos(theta_3))
const _Plane p2 = { 0,0 * cos(PI / 2 - theta_3) + (-1) * (-sin(PI / 2 - theta_3)),0 * sin(PI / 2 - theta_3) + (-1) * cos(PI / 2 - theta_3),0 };
const int LINE_NUMBER = 100;

std::vector<cv::Point3f> Calculate_Optical_Image_Center_Point();
float Calculate_Laser_Lens_Angle();
_Vec_Point_Pair Random_Generate_Point_Pair(cv::Point3f optical_center, _Plane p1, _Plane p2, int number = LINE_NUMBER);
std::vector<cv::Point3f> Coordinate_System_conversion_to_Image_Center(vector<cv::Point3f> points, cv::Point3f optical_center);
std::vector<cv::Point3f> Simulated_Image_Distortion(vector<cv::Point3f> points);
std::vector<cv::Point2f>Point3f_To_Point2f(std::vector<cv::Point3f>Points);

//! 以激光平面与成像平面与镜头平面的交点作为空间直角坐标系原点
//! 激光平面垂直与y轴且过原点
int main() {


    //! 求 光心位置 和 像平面原点
    std::vector<cv::Point3f> point_center = Calculate_Optical_Image_Center_Point();

    //! 求 光心位置 
    cv::Point3f optical_center = point_center[0];
    //! 求 像平面原点
    cv::Point3f image_center = point_center[1];

    //! 求镜片与激光夹角
    theta_4 = Calculate_Laser_Lens_Angle();

    //! 求交点
    _Vec_Point_Pair PointPair = Random_Generate_Point_Pair(optical_center, p1, p2);
    //! 修正像平面坐标系
    std::vector<cv::Point3f>Correct_Point = Coordinate_System_conversion_to_Image_Center(PointPair.p2, image_center);

    //! 像平面叠加即便模型
    std::vector<cv::Point3f>Distortion_Point = Simulated_Image_Distortion(Correct_Point);

    //! 由Point3f转为Point2f，放弃z轴数据
    std::vector<cv::Point2f>Distortion_Points2f = Point3f_To_Point2f(Distortion_Point);

    //for (int i = 0; i < Distortion_Point.size(); i++) {
    //	cout << Distortion_Point[i].x << " ";
    //}
    //cout << endl;
    //for (int i = 0; i < Distortion_Point.size(); i++) {
    //	cout << Distortion_Point[i].y << " ";
    //}
    //cout << endl;

    //for (int i = 0; i < Distortion_Point.size(); i++) {
    //	cout << Distortion_Point[i].z << " ";
    //}

    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p1[i].x << " ";
    //}
    //cout << endl; cout << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p1[i].y << " ";
    //}cout << endl; cout << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p1[i].z << " ";
    //}

    //cout << endl; cout << endl; cout << endl; cout << endl;

    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p2[i].x << " ";
    //}
    //cout << endl; cout << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p2[i].y << " ";
    //}cout << endl; cout << endl;
    //for (int i = 0; i < PointPair.p1.size(); i++) {
    //    cout << PointPair.p2[i].z << " ";
    //}

    std::vector < std::vector<cv::Point3f>> obj;
    std::vector < std::vector<cv::Point2f>> img;

    //obj[0].push_back({ 0, 0, 0 });
    //obj[0].push_back({ 0, 3, 0 });
    //obj[0].push_back({ 0, 6, 0 });
    //obj[0].push_back({ 0, 9, 0 });
    //obj[0].push_back({ 0, 12, 0 });

    //img[0].push_back({ 448.48288, 343.86087 });
    //img[0].push_back({ 478.1792, 344.48053 });
    //img[0].push_back({ 507.76147, 345.14603 });
    //img[0].push_back({ 537.19342, 345.77951 });
    //img[0].push_back({ 566.50525, 346.4072 });
    obj.push_back(PointPair.p1);
    img.push_back(Distortion_Points2f);
    //! ----------------------------------------------------------------------------------------------------------------
    std::vector<cv::Point3f> object_points_seq;	//存储所有图片的世界坐标
    float data[] = { 35, 0, 5000,0,0.35,6000,0,0,0 };
    cv::Mat cameraMatrix(3, 3, CV_32FC1, data);
    // 从数组初始化

    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecsMat;
    std::vector<cv::Mat> tvecsMat;
    cv::Size image_size;
    image_size.height = 10000;
    image_size.width = 12000;
    ///*运行标定函数*/
    float err_first = 0.0;
    err_first = cv::calibrateCamera(
        obj,
        img,
        image_size,
        cameraMatrix,
        distCoeffs,
        rvecsMat,
        tvecsMat,
        cv::CALIB_TILTED_MODEL);
    ///*输出内参数*/
    //std::cout << "cameraMatrix:" << std::endl;
    //std::cout << cameraMatrix << std::endl;
    //std::cout << "distCoeffs:" << std::endl;
    //std::cout << distCoeffs << std::endl;

    ///*输出外参数*/
    //std::cout << "rvecsMat:" << std::endl;
    //std::cout << rvecsMat[0] << std::endl;
    //std::cout << "tvecsMat:" << std::endl;
    //std::cout << tvecsMat[0] << std::endl;
    return 0;
}


//! 计算光心位置 和 像原点
std::vector<cv::Point3f> Calculate_Optical_Image_Center_Point() {

    //! P[0]为光心 P[1]为像原点
    std::vector<cv::Point3f> P;

    //! 光心的坐标为（0, a, e）
    //! e[未知] 为光心距离x-z平面的距离


    float distance_e = -(((sin(theta_2) * (fabs(distance_b) + fabs(distance_a) / sin(theta_1))) / sin(theta_3)) - fabs(distance_a) / tan(theta_1));
    //! 镜头光心坐标
    cv::Point3f Optical_Center = { 0, distance_a, distance_e };

    P.push_back(Optical_Center);

    float theta_8 = (180 - (360 - 90 - theta_3 / PI * 180 - theta_2 / PI * 180)) / 180 * PI;

    cv::Point3f Image_Center = { 0.0, -(fabs(distance_a) + fabs(distance_b) * cos(theta_8)) ,-(fabs(distance_e) - fabs(distance_b) * sin(theta_8)) };

    P.push_back(Image_Center);

    return P;
}

//! 计算镜头平面与激光平面夹角
float Calculate_Laser_Lens_Angle() {
    //float theta_4 = atan(1 / (((fabs(distance_b) * sin(theta_2)) / (fabs(distance_a) * sin(theta_3))) + (sin(theta_2) / (sin(theta_1) * sin(theta_3))) - (1 / tan(theta_1))));
    //float theta_4 = atan(fabs(distance_a) * tan(theta_1) * sin(theta_3)) / ((tan(theta_1) * sin(theta_2) * (fabs(distance_b) + fabs(distance_a) / sin(theta_1))) - (fabs(distance_a) * sin(theta_3)));
    float theta_4 = atan(fabs(distance_a) / (((sin(theta_2) * (fabs(distance_b) + fabs(distance_a) / sin(theta_1))) / sin(theta_3)) - fabs(distance_a) / tan(theta_1)));
    /*float theta_4 = atan(
        (fabs(distance_a))
        / (
            (fabs(distance_b) * sin(theta_2) / sin(theta_3))
            + (fabs(distance_a) * sin(theta_2) / (sin(theta_3) * sin(theta_1)))
            - (fabs(distance_a) * cos(theta_1) / sin(theta_1))
            )
    );*/

    //!分步计算
    /*
    float d = fabs(distance_a) / sin(theta_1);
    float g = fabs(distance_a) / tan(theta_1);
    float e = (fabs(distance_b) + d) * sin(theta_2) / sin(theta_3) - g;
    float theta_4 = atan(fabs(distance_a) / e);
    */
    return theta_4;
}

//! 判断 P1 P2 相交
bool Is_Intersect(_Plane P1, _Plane P2, _Line L) {
    if (P1.A * L.a + P1.B * L.b + P1.C * L.c == 0 || P2.A * L.a + P2.B * L.b + P2.C * L.c == 0) {
        return false;
    }
    return true;
}

//! 求线面交点 - 世界坐标
cv::Point3f Calculate_Line_Plane_Intersection_Point(cv::Point3f optical_center, _Plane P, _Line Line) {

    cv::Point3f point = { 0.0,0.0,0.0 };

    point.x =
        optical_center.x - Line.a * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);
    point.y =
        optical_center.y - Line.b * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);
    point.z =
        optical_center.z - Line.c * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);

    return point;
}

//! 随机生成的直线 并要求穿光心后得到两个面交点
_Vec_Point_Pair Random_Generate_Point_Pair(cv::Point3f optical_center, _Plane p1, _Plane p2, int number) {
    //! 随机生成 49 条直线且与 p1 p2 相交的
    _Vec_Point_Pair PointPair;
    _Line Line = { 0.0,0.0,0.0 };
    for (int i = 0; i < number; i++) {

        Line.a = rand() % 100 - 50;
        Line.b = rand() % 100 - 50;
        Line.c = rand() % 100 - 50;
        //!限制直线交点只允许出现在z的负半轴并且与俩平面存在交点
        if (Line.b / Line.c < 0 && Is_Intersect(p1, p2, Line)) {
            PointPair.p1.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, p1, Line));
            PointPair.p2.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, p2, Line));
        }
        else {
            i--;
        }
    }
    return PointPair;
}

//! 将传感器平面转为像坐标系
std::vector<cv::Point3f> Coordinate_System_conversion_to_Image_Center(vector<cv::Point3f> points, cv::Point3f image_center) {

    vector<cv::Point3f> RT_points;
    //! 像平面坐标由世界坐标系转为传感器（像平面）坐标系，以橡心为原点
    for (int i = 0; i < points.size(); i++) {

        //! 平移
        points[i].x = points[i].x - image_center.x;
        points[i].y = points[i].y - image_center.y;
        points[i].z = points[i].z - image_center.z;

        cv::Point3f point = { points[i].x,0,0 };

        //! 旋转 交点逆时针旋转 theta_9 个弧度
        point.y = points[i].y * cos(theta_9) + points[i].z * sin(theta_9);
        point.z = -points[i].y * sin(theta_9) + points[i].z * cos(theta_9);
        RT_points.push_back(point);
    }

    return RT_points;
}

//! 添加成像平面p2径向畸变
std::vector<cv::Point3f> Simulated_Image_Distortion(vector<cv::Point3f> points) {
    return points;
    //! 设置畸变因子
    float k1 = 0.004408749451738147;
    float k2 = 0.0003288813627739166;
    float k3 = 0.00008529518568999;
    //! 设置内参
    //! fx = 相机焦距 * 采样比（每毫米对应的像素 仿真模型中设为1）
    float fx = 35, fy = 35;


    for (int i = 0; i < points.size(); i++) {
        //! 计算距离
        //! 转到相机坐标系下（由于当前是以图像中心为坐标原点，故不需要减去cx cy）
        points[i].x = points[i].x / fx;
        points[i].y = points[i].y / fy;
        float Distance_Center = sqrt(pow(points[i].x, 2) + pow(points[i].y, 2));
        //! 1 + xxx 为桶形失真，  1 - xxx 为枕形失真
        float lambda = 1 + k1 * pow(Distance_Center, 2 * 1) + k2 * pow(Distance_Center, 2 * 2) + k3 * pow(Distance_Center, 2 * 3);
        points[i].x = points[i].x * lambda * fx;
        points[i].y = points[i].y * lambda * fy;
    }
    //! 返回添加完畸变的vector坐标

}

//!三维坐标转二维坐标 
std::vector<cv::Point2f>Point3f_To_Point2f(std::vector<cv::Point3f>Points) {
    std::vector<cv::Point2f> Points2f;
    cv::Point2f Point2f;
    for (int i = 0; i < Points.size(); i++) {
        Point2f.x = Points[i].x;
        Point2f.y = Points[i].y;
        Points2f.push_back(Point2f);
    }
    return Points2f;
}