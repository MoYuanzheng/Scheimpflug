#pragma once

#include<stdio.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<algorithm>
#include<opencv2/opencv.hpp>

using namespace std;

const double PI = 4 * atan(1);
//! 平面结构
struct _Plane {
    double A;
    double B;
    double C;
    double D;
};
//! 直线结构
struct _Line
{
    double a;
    double b;
    double c;
};
//! 点对
struct _Vec_Point_Pair
{
    //! 位于激光平面的坐标vec
    std::vector<cv::Point3f> p1;

    //! 位于传感器(图像)平面的坐标vec
    std::vector<cv::Point3f> p2;
};

//! theta_1[已知] 激光平面与光轴夹角
const double theta_1 = 45.0 / 180 * PI;

//! theta_2[未知] 光轴与传感器平面夹角
double theta_2 = 0;

//! theta_3[未知] 激光平面与传感器平面夹角
double theta_3 = 0;

//! theta_4[未知] 激光平面与镜头平面夹角
double theta_4 = PI / 2 - theta_1;

//! theta_9[未知] 坐标系旋转角度
double theta_9 = 0;

//! a[已知] 为激光平面至镜头光心的距离
//! b[已知] 为镜头光心至传感器的距离
const double distance_a = 50.0, distance_b = 35.0;
const double distance_e = distance_a * tan(theta_1);
const double distance_f = distance_b * cos(PI / 2 - theta_1);
const double distance_k = distance_b * sin(PI / 2 - theta_1);
const double distance_h = distance_e - distance_k;
const double distance_g = distance_a / tan(theta_1);

//! p1为激光平面
const _Plane p1 = { 0,1.0,0,0 };

//! 靶面 -> 尺寸 -> 单位毫米
const cv::Size2f Sensor_Size = { 4.608f,9.216f };
//! 靶面 -> 尺寸 -> 单位像素
const cv::Size Image_Size = { 1024,2048 };
//! 靶面 -> 像元 -> 尺寸 -> 单位毫米
const double Pixel_length = 0.0045;
//! 靶面 -> 角点
vector<cv::Point3f> P2_4_Corner;//! LU RU LD RD
//! 靶面 -> 方程
_Plane P2 = { 0.0,0.0,0.0,0.0 };

//! 生成直线数量
const int LINE_NUMBER = 0;

//! 设置畸变因子
const double k1 = 0.04408749451738147;
const double k2 = 0.003288813627739166;
const double k3 = 0.0008529518568999;

const double Rows_Sample_Rate = 1 / Pixel_length;
const double Cols_Sample_Rate = 1 / Pixel_length;

//! ===============================================================================================
//! 函数声明部分
std::vector<cv::Point3f> Calculate_Optical_Sensor_Center_Point_2();
double Calculate_Laser_Sensor_Angle();
_Vec_Point_Pair Random_Generate_Point_Pair(cv::Point3f optical_center, _Plane p1, _Plane p2, vector<_Line> Line_Direction_Vector_Range, int number = LINE_NUMBER);
std::vector<cv::Point3f> Calculate_Special_Points(cv::Point3f optical_center, _Plane plane, vector<cv::Point3f> Corner_Points);
std::vector<cv::Point3f> Coordinate_System_conversion_to_Pixel_P1(std::vector<cv::Point3f> Points, cv::Point3f Origin);
std::vector<cv::Point2f> Coordinate_System_conversion_to_Pixel_P2(vector<cv::Point3f> points);
std::vector<cv::Point3f> Simulated_Image_Distortion(vector<cv::Point3f> points);
std::vector<_Line> Limit_P2_Intersection_Point_Range(cv::Point3f Optocal_Center, vector<cv::Point3f> P2_4_Corner);
cv::Size2f Calculate_FOV_Max(std::vector<cv::Point3f> Cornor_Points);
cv::Point3f Calculate_Origin(std::vector<cv::Point3f> P1_Cornor_Points);
void Calculate_P2_Corner(cv::Point3f Sensor_Center, cv::Size2f Sensor_Size, double theta_9);
cv::Point3f Calculate_Line_Plane_Intersection_Point(cv::Point3f optical_center, _Plane P, _Line Line);