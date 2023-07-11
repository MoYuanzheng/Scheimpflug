#pragma once

#include<stdio.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<algorithm>
#include<opencv2/opencv.hpp>

using namespace std;

const float PI = 4 * atan(1);
//! 平面结构
struct _Plane {
    float A;
    float B;
    float C;
    float D;
};
//! 直线结构
struct _Line
{
    float a;
    float b;
    float c;
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
float distance_e;

//! p1为激光平面
const _Plane p1 = { 0,1.0,0,0 };

//! 设定放置物体的地面相对于原点有100mm距离
const float _floor = 100.0;

//! 激光平面物体限制 高 50mm 宽 100mm
//const float p1_Height = 50.0, p1_Width = 100.0;
const cv::Size2f P_1_Size = { 100.0,50.0 };


//! 相机的像素尺寸
const cv::Size Image_Size = { 2448,1024 };

//! p2为成像平面（传感器平面）
//! 假设p2平面是由方向向量为(0,0,-1) 对应 (i,j,k) 的平面绕x轴逆时针旋转PI/2 - theta_3个弧度得到的平面，那么宣传后的法向量为
//!														(0,j*cos(theta_3)−k*sin(theta_3),j*sin(theta_3)+k*cos(theta_3))
const _Plane p2 = { 0,0 * cos(PI / 2 - theta_3) + (-1) * (-sin(PI / 2 - theta_3)),0 * sin(PI / 2 - theta_3) + (-1) * cos(PI / 2 - theta_3),0 };

//! 生成直线数量
const int LINE_NUMBER = 12000;

//! 设置畸变因子
const float k1 = 0.04408749451738147;
const float k2 = 0.003288813627739166;
const float k3 = 0.0008529518568999;

//! 设置内参
//! fx = 相机焦距 * 采样比（每毫米对应的像素 10代表1mm距离可采集10个pixel）
const float fx = 35.0 * 10, fy = 35.0 * 12;

//! 函数声明部分
std::vector<cv::Point3f> Calculate_Optical_Image_Center_Point();
float Calculate_Laser_Lens_Angle();
_Vec_Point_Pair Random_Generate_Point_Pair(cv::Point3f optical_center, _Plane p1, _Plane p2, vector<_Line> Line_Direction_Vector_Range, int number = LINE_NUMBER);
std::vector<cv::Point3f> Coordinate_System_conversion_to_Image_Center(vector<cv::Point3f> points, cv::Point3f optical_center);
std::vector<cv::Point2f> Coordinate_System_conversion_to_Pixel(vector<cv::Point3f> Distortion_Point, cv::Size Image_Size);
std::vector<cv::Point3f> Simulated_Image_Distortion(vector<cv::Point3f> points);
std::vector<cv::Point3f>Point3f_Inversion_Z_Y(std::vector<cv::Point3f> Points);
vector<_Line> Limit_p1_Intersection_Point_Range(cv::Point3f Optocal_Center, cv::Size2f P_1_Size);