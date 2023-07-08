
#include<stdio.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<algorithm>
using namespace std;

const double PI = 4 * atan(1);

struct _Plane {
    double A;
    double B;
    double C;
    double D;
};

struct _Line
{
    double a;
    double b;
    double c;
};

struct _Point
{
    double x;
    double y;
    double z;
};

struct _Vec_Point_Pair
{
    //! 位于激光平面的坐标vec
    std::vector<_Point> p1;
    //! 位于传感器(图像)平面的坐标vec
    std::vector<_Point> p2;
};

//! theta_1[已知] 激光平面与光轴夹角
const double theta_1 = 45.0 / 180 * PI;

//! theta_2[已知] 光轴与传感器平面夹角
const double theta_2 = 60.0 / 180 * PI;

//! theta_3[已知] 激光平面与传感器平面夹角
const double theta_3 = 75.0 / 180 * PI;

//! theta_4[未知] 激光平面与镜头平面夹角
double theta_4;

//! theta_9[未知] 坐标系旋转角度
double theta_9 = PI / 2 - theta_3;


//! a[已知] 为激光平面至镜头光心的距离
//! b[已知] 为镜头光心至传感器的距离
const double distance_a = -50.0, distance_b = -35.0;

//! p1为激光平面
const _Plane p1 = { 0,1.0,0,0 };


//! p2为成像平面（传感器平面）
//! 假设p2平面是由方向向量为(0,0,-1) 对应 (i,j,k) 的平面绕x轴逆时针旋转PI/2 - theta_3个弧度得到的平面，那么宣传后的法向量为
//!														(0,j*cos(theta_3)−k*sin(theta_3),j*sin(theta_3)+k*cos(theta_3))
const _Plane p2 = { 0,0 * cos(PI / 2 - theta_3) + (-1) * (-sin(PI / 2 - theta_3)),0 * sin(PI / 2 - theta_3) + (-1) * cos(PI / 2 - theta_3),0 };


std::vector<_Point> Calculate_Optical_Image_Center_Point();
double Calculate_Laser_Lens_Angle();
_Vec_Point_Pair Random_Generate_Point_Pair(_Point optical_center, _Plane p1, _Plane p2, int number = 49);
std::vector<_Point> Coordinate_System_conversion_to_Image_Center(vector<_Point> points, _Point optical_center);
std::vector<_Point> Simulated_Image_Distortion(vector<_Point> points);

//! 以激光平面与成像平面与镜头平面的交点作为空间直角坐标系原点
//! 激光平面垂直与y轴且过原点
int main() {


    //! 求 光心位置 和 像平面原点
    std::vector<_Point> point_center = Calculate_Optical_Image_Center_Point();

    //! 求 光心位置 
    _Point optical_center = point_center[0];
    //! 求 像平面原点
    _Point image_center = point_center[1];

    //! 求镜片与激光夹角
    theta_4 = Calculate_Laser_Lens_Angle();

    //! 求交点
    _Vec_Point_Pair PointPair = Random_Generate_Point_Pair(optical_center, p1, p2);
    //! 修正像平面坐标系
    std::vector<_Point>Correct_Point = Coordinate_System_conversion_to_Image_Center(PointPair.p2, image_center);
    //! 像平面叠加即便模型

    std::vector<_Point>Distortion_Point = Simulated_Image_Distortion(Correct_Point);

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
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p1[i].x << " ";
    }
    cout << endl; cout << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p1[i].y << " ";
    }cout << endl; cout << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p1[i].z << " ";
    }

    cout << endl; cout << endl; cout << endl; cout << endl;

    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p2[i].x << " ";
    }
    cout << endl; cout << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p2[i].y << " ";
    }cout << endl; cout << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p2[i].z << " ";
    }
    return 0;
}


//! 计算光心位置 和 像原点
std::vector<_Point> Calculate_Optical_Image_Center_Point() {

    //! P[0]为光心 P[1]为像原点
    std::vector<_Point> P;

    //! 光心的坐标为（0, a, e）
    //! e[未知] 为光心距离x-z平面的距离


    double distance_e = -(((sin(theta_2) * (fabs(distance_b) + fabs(distance_a) / sin(theta_1))) / sin(theta_3)) - fabs(distance_a) / tan(theta_1));
    //! 镜头光心坐标
    _Point Optical_Center = { 0, distance_a, distance_e };

    P.push_back(Optical_Center);

    double theta_8 = (180 - (360 - 90 - theta_3 / PI * 180 - theta_2 / PI * 180)) / 180 * PI;

    _Point Image_Center = { 0.0, -(fabs(distance_a) + fabs(distance_b) * cos(theta_8)) ,-(fabs(distance_e) - fabs(distance_b) * sin(theta_8)) };

    P.push_back(Image_Center);

    return P;
}

//! 计算镜头平面与激光平面夹角
double Calculate_Laser_Lens_Angle() {
    //double theta_4 = atan(1 / (((fabs(distance_b) * sin(theta_2)) / (fabs(distance_a) * sin(theta_3))) + (sin(theta_2) / (sin(theta_1) * sin(theta_3))) - (1 / tan(theta_1))));
    //double theta_4 = atan(fabs(distance_a) * tan(theta_1) * sin(theta_3)) / ((tan(theta_1) * sin(theta_2) * (fabs(distance_b) + fabs(distance_a) / sin(theta_1))) - (fabs(distance_a) * sin(theta_3)));
    double theta_4 = atan(fabs(distance_a) / (((sin(theta_2) * (fabs(distance_b) + fabs(distance_a) / sin(theta_1))) / sin(theta_3)) - fabs(distance_a) / tan(theta_1)));
    /*double theta_4 = atan(
        (fabs(distance_a))
        / (
            (fabs(distance_b) * sin(theta_2) / sin(theta_3))
            + (fabs(distance_a) * sin(theta_2) / (sin(theta_3) * sin(theta_1)))
            - (fabs(distance_a) * cos(theta_1) / sin(theta_1))
            )
    );*/

    //!分步计算
    /*
    double d = fabs(distance_a) / sin(theta_1);
    double g = fabs(distance_a) / tan(theta_1);
    double e = (fabs(distance_b) + d) * sin(theta_2) / sin(theta_3) - g;
    double theta_4 = atan(fabs(distance_a) / e);
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
_Point Calculate_Line_Plane_Intersection_Point(_Point optical_center, _Plane P, _Line Line) {

    _Point point = { 0.0,0.0,0.0 };

    point.x =
        optical_center.x - Line.a * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);
    point.y =
        optical_center.y - Line.b * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);
    point.z =
        optical_center.z - Line.c * (P.A * optical_center.x + P.B * optical_center.y + P.C * optical_center.z + P.D) / (P.A * Line.a + P.B * Line.b + P.C * Line.c);

    return point;
}

//! 随机生成的直线 并要求穿光心后得到两个面交点
_Vec_Point_Pair Random_Generate_Point_Pair(_Point optical_center, _Plane p1, _Plane p2, int number) {
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
std::vector<_Point> Coordinate_System_conversion_to_Image_Center(vector<_Point> points, _Point image_center) {

    vector<_Point> RT_points;
    //! 像平面坐标由世界坐标系转为传感器（像平面）坐标系，以橡心为原点
    for (int i = 0; i < points.size(); i++) {

        //! 平移
        points[i].x = points[i].x - image_center.x;
        points[i].y = points[i].y - image_center.y;
        points[i].z = points[i].z - image_center.z;

        _Point point = { points[i].x,0,0 };

        //! 旋转 交点逆时针旋转 theta_9 个弧度
        point.y = points[i].y * cos(theta_9) + points[i].z * sin(theta_9);
        point.z = -points[i].y * sin(theta_9) + points[i].z * cos(theta_9);
        RT_points.push_back(point);
    }

    return RT_points;
}

//! 添加成像平面p2径向畸变
std::vector<_Point> Simulated_Image_Distortion(vector<_Point> points) {

    //! 设置畸变因子
    double k1 = 0.04408749451738147;
    double k2 = -0.3288813627739166;
    double k3 = 1.08529518568999;
    //! 设置内参
    double fx = 2560.0, fy = 2560.0;


    for (int i = 0; i < points.size(); i++) {
        //! 计算距离
        //! 转到相机坐标系下（由于当前是以图像中心为坐标原点，故不需要减去cx cy）
        points[i].x = points[i].x / fx;
        points[i].y = points[i].y / fy;
        double Distance_Center = sqrt(pow(points[i].x, 2) + pow(points[i].y, 2));
        //! 1 + xxx 为桶形失真，  1 - xxx 为枕形失真
        double lambda = 1 + k1 * pow(Distance_Center, 2 * 1) + k2 * pow(Distance_Center, 2 * 2) + k3 * pow(Distance_Center, 2 * 3);
        points[i].x = points[i].x * lambda * fx;
        points[i].y = points[i].y * lambda * fy;
    }
    //! 返回添加完畸变的vector坐标
    return points;
}
