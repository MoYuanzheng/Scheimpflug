#include"head.h"

//! 以激光平面与成像平面与镜头平面的交点作为空间直角坐标系原点
//! 激光平面垂直与y轴且过原点
int main() {
    //! 求 光心位置 和 像平面原点
    std::vector<cv::Point3f> point_center = Calculate_Optical_Image_Center_Point();

    //! 求 光心位置 
    cv::Point3f Optocal_Center = point_center[0];
    //! 求 像平面原点
    cv::Point3f image_center = point_center[1];

    //! 求镜片与激光夹角
    theta_4 = Calculate_Laser_Lens_Angle();

    //! 限制直线方向
    vector<_Line> Line_Direction_Vector_Range = Limit_p1_Intersection_Point_Range(Optocal_Center, P_1_Size);

    //! 求随机线面交点
    _Vec_Point_Pair PointPair = Random_Generate_Point_Pair(Optocal_Center, p1, p2, Line_Direction_Vector_Range);

    //! 求固定四个角线 面的交点
    std::vector<cv::Point3f> PointPair_Fix_Corner = Fix_Generate_Point_Pair(Optocal_Center, p2, P1_4_Corner);

    //! 将固定点对追加到随机点对中
    PointPair.p1.insert(std::end(PointPair.p1), std::begin(P1_4_Corner), std::end(P1_4_Corner));
    PointPair.p2.insert(std::end(PointPair.p2), std::begin(PointPair_Fix_Corner), std::end(PointPair_Fix_Corner));


    //! 改变像平面坐标系 以像平面角点为原点 建立像素坐标
    std::vector<cv::Point2f>Image_Pixel_Points = Coordinate_System_conversion_to_Pixel_P2(PointPair.p2);

    //! 图像平面叠加畸变模型
    //std::vector<cv::Point3f>Distortion_Point = Simulated_Image_Distortion(Image_Pixel_Points);

    //! 激光平面模拟真实标定板提取角点
    std::vector<cv::Point3f>P1_Pixel_Points = Coordinate_System_conversion_to_Pixel_P1(PointPair.p1);

    std::vector < std::vector<cv::Point3f>> obj;
    std::vector < std::vector<cv::Point2f>> img;

    obj.push_back(P1_Pixel_Points);
    img.push_back(Image_Pixel_Points);
    //! ----------------------------------------------------------------------------------------------------------------
    //float data[] = { 350, 0, 1224,0,420,512,0,0,1 };
    //cv::Mat cameraMatrix(3, 3, CV_32FC1, data);
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecsMat;
    std::vector<cv::Mat> tvecsMat;


    ///*运行标定函数*/
    float err_first = 0.0;
    err_first = cv::calibrateCamera(obj, img, Image_Size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);

    ///*输出内参数*/
    std::cout << "cameraMatrix:" << std::endl;
    std::cout << cameraMatrix << std::endl;
    std::cout << "distCoeffs:" << std::endl;
    std::cout << distCoeffs << std::endl;

    ///*输出外参数*/
    std::cout << "rvecsMat:" << std::endl;
    std::cout << rvecsMat[0] << std::endl;
    std::cout << "tvecsMat:" << std::endl;
    std::cout << tvecsMat[0] << std::endl;
    return 0;
}


//! 计算光心位置 和 像原点
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-Ylfmd5KirojBOrxxvlJchDQqnHc
std::vector<cv::Point3f> Calculate_Optical_Image_Center_Point() {

    //! P[0]为光心 P[1]为像原点
    std::vector<cv::Point3f> P;

    //! 光心的坐标为（0, a, e）
    //! e[未知] 为光心距离x-z平面的距离


    distance_e = -(((sin(theta_2) * (fabs(distance_b) + fabs(distance_a) / sin(theta_1))) / sin(theta_3)) - fabs(distance_a) / tan(theta_1));
    //! 镜头光心坐标
    cv::Point3f Optical_Center = { 0, distance_a, distance_e };

    P.push_back(Optical_Center);

    float theta_8 = (180 - (360 - 90 - theta_3 / PI * 180 - theta_2 / PI * 180)) / 180 * PI;

    cv::Point3f Image_Center = { 0.0, -(fabs(distance_a) + fabs(distance_b) * cos(theta_8)) ,-(fabs(distance_e) - fabs(distance_b) * sin(theta_8)) };

    P.push_back(Image_Center);

    return P;
}

//! 计算镜头平面与激光平面夹角
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-Ylfmd5KirojBOrxxvlJchDQqnHc
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

//! 限制直线处在目标范围内
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-OoJ1dbLuGokeihxfuZocJHaQnUd
vector<_Line> Limit_p1_Intersection_Point_Range(cv::Point3f Optocal_Center, cv::Size2f P_1_Size) {
    cv::Point3f p1_Limit_Left_Top = { P_1_Size.width / 2, 0, -_floor + P_1_Size.height };
    cv::Point3f p1_Limit_Left_Low = { P_1_Size.width / 2, 0, -_floor };
    cv::Point3f p1_Limit_Right_Top = { -P_1_Size.width / 2, 0, -_floor + P_1_Size.height };
    cv::Point3f p1_Limit_Right_Low = { -P_1_Size.width / 2, 0, -_floor };
    vector<_Line> Line_Direction_Vector_Range;
    _Line temp = {
        (Optocal_Center - p1_Limit_Left_Top).x,
        (Optocal_Center - p1_Limit_Left_Top).y,
        (Optocal_Center - p1_Limit_Left_Top).z
    };
    Line_Direction_Vector_Range.push_back(temp);

    temp = {
        (Optocal_Center - p1_Limit_Left_Low).x,
        (Optocal_Center - p1_Limit_Left_Low).y,
        (Optocal_Center - p1_Limit_Left_Low).z
    };
    Line_Direction_Vector_Range.push_back(temp);

    temp = {
        (Optocal_Center - p1_Limit_Right_Top).x,
        (Optocal_Center - p1_Limit_Right_Top).y,
        (Optocal_Center - p1_Limit_Right_Top).z
    };
    Line_Direction_Vector_Range.push_back(temp);

    temp = {
        (Optocal_Center - p1_Limit_Right_Low).x,
        (Optocal_Center - p1_Limit_Right_Low).y,
        (Optocal_Center - p1_Limit_Right_Low).z
    };
    Line_Direction_Vector_Range.push_back(temp);

    return Line_Direction_Vector_Range;
}

//! 求线面交点 - 世界坐标
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-OoJ1dbLuGokeihxfuZocJHaQnUd
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
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-OoJ1dbLuGokeihxfuZocJHaQnUd
_Vec_Point_Pair Random_Generate_Point_Pair(cv::Point3f optical_center, _Plane p1, _Plane p2, vector<_Line> Line_Direction_Vector_Range, int number) {
    //! 随机生成 49 条直线且与 p1 p2 相交的
    _Vec_Point_Pair PointPair;
    _Line Line = { 0.0,0.0,0.0 };
    for (int i = 0; i < number; i++) {

        int a_max = std::max({ Line_Direction_Vector_Range[0].a,Line_Direction_Vector_Range[1].a,Line_Direction_Vector_Range[2].a,Line_Direction_Vector_Range[3].a });
        int a_min = std::min({ Line_Direction_Vector_Range[0].a,Line_Direction_Vector_Range[1].a,Line_Direction_Vector_Range[2].a,Line_Direction_Vector_Range[3].a });

        int b_max = std::max({ Line_Direction_Vector_Range[0].b,Line_Direction_Vector_Range[1].b,Line_Direction_Vector_Range[2].b,Line_Direction_Vector_Range[3].b });
        int b_min = std::min({ Line_Direction_Vector_Range[0].b,Line_Direction_Vector_Range[1].b,Line_Direction_Vector_Range[2].b,Line_Direction_Vector_Range[3].b });

        int c_max = std::max({ Line_Direction_Vector_Range[0].c,Line_Direction_Vector_Range[1].c,Line_Direction_Vector_Range[2].c,Line_Direction_Vector_Range[3].c });
        int c_min = std::min({ Line_Direction_Vector_Range[0].c,Line_Direction_Vector_Range[1].c,Line_Direction_Vector_Range[2].c,Line_Direction_Vector_Range[3].c });

        //  srand(time(NULL) + i);
        double a_rand = (double)rand() / RAND_MAX;  // 生成介于 0 和 1 之间的随机数
        Line.a = a_rand * (a_max - a_min) + (a_min);  // 缩放到 (a_min, a_max) 区间

        //  srand(time(NULL) + int(Line.a) + i);
        double b_rand = (double)rand() / RAND_MAX;  // 生成介于 0 和 1 之间的随机数
        Line.b = b_rand * (b_max - b_min) + (b_min);  // 缩放到 (b_min, b_max) 区间

        // srand(time(NULL) + int(Line.b) + i);
        double c_rand = (double)rand() / RAND_MAX;  // 生成介于 0 和 1 之间的随机数
        Line.c = c_rand * (c_max - c_min) + (c_min);  // 缩放到 (c_min, c_max) 区间

        //!不允许出现直线与P1或P2平行的情况
        if (Is_Intersect(p1, p2, Line)) {
            PointPair.p1.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, p1, Line));
            PointPair.p2.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, p2, Line));
        }
        else {
            i--;
        }
    }

    return PointPair;
}

//! 找到像平面角点 并生成对应点对
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-EC6udrjTPoNfAMxyf4CctmKcnfg
std::vector<cv::Point3f>  Fix_Generate_Point_Pair(cv::Point3f optical_center, _Plane p2, vector<cv::Point3f> P1_4_Corner) {
    std::vector<cv::Point3f> Fix_Point_Pair;

    _Line Line_LU = { P1_4_Corner[0].x - optical_center.x,P1_4_Corner[0].y - optical_center.y,P1_4_Corner[0].z - optical_center.z };
    _Line Line_RU = { P1_4_Corner[1].x - optical_center.x,P1_4_Corner[1].y - optical_center.y,P1_4_Corner[1].z - optical_center.z };
    _Line Line_LD = { P1_4_Corner[2].x - optical_center.x,P1_4_Corner[2].y - optical_center.y,P1_4_Corner[2].z - optical_center.z };
    _Line Line_RD = { P1_4_Corner[3].x - optical_center.x,P1_4_Corner[3].y - optical_center.y,P1_4_Corner[3].z - optical_center.z };

    Fix_Point_Pair.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, p2, Line_LU));
    Fix_Point_Pair.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, p2, Line_RU));
    Fix_Point_Pair.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, p2, Line_LD));
    Fix_Point_Pair.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, p2, Line_RD));

    return Fix_Point_Pair;
}

//! 将传感器平面转为以角点建立坐标系（旋转、平移及像素化）
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-OoJ1dbLuGokeihxfuZocJHaQnUd
std::vector<cv::Point2f> Coordinate_System_conversion_to_Pixel_P2(vector<cv::Point3f> points) {

    //! 像平面坐标由世界坐标系转为传感器（像平面）坐标系，以橡心为原点
    std::vector<cv::Point3f> points_R;
    for (int i = 0; i < points.size(); i++) {

        points_R.push_back({ points[i].x,0,0 });

        //! 旋转 交点逆时针旋转 theta_9 个弧度
        points_R[i].y = points[i].y * cos(theta_9) + points[i].z * sin(theta_9);
        points_R[i].z = -points[i].y * sin(theta_9) + points[i].z * cos(theta_9);
    }

    //! 平移至角点 建立像素坐标系
    std::vector<cv::Point3f> points_RT;
    for (int i = 0; i < points_R.size(); i++) {

        points_RT.push_back({
            (points_R[i].y - points_R[LINE_NUMBER + 1].y),
            -(points_R[i].x - points_R[LINE_NUMBER + 1].x),
            points_R[i].z });
    }

    //! 转像素  按照采样比进行像素转换
    vector<cv::Point2f> Image_Pixel_Points;
    for (int i = 0; i < points_RT.size(); i++) {
        Image_Pixel_Points.push_back({
            points_RT[i].x * Y_Sample_Rate ,
            points_RT[i].y * X_Sample_Rate
            });
    }
    return Image_Pixel_Points;
}

//! 添加成像平面p2径向畸变
std::vector<cv::Point3f> Simulated_Image_Distortion(vector<cv::Point3f> points) {

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
    return points;
}

//! P1平面像素化
std::vector<cv::Point3f> Coordinate_System_conversion_to_Pixel_P1(std::vector<cv::Point3f> Points) {
    std::vector<cv::Point3f> P1_Pixel_Points;

    for (int i = 0; i < Points.size(); i++) {

        //! 平移
        Points[i].x = Points[i].x - P1_4_Corner[0].x;
        Points[i].y = Points[i].y - P1_4_Corner[0].y;
        Points[i].z = Points[i].z - P1_4_Corner[0].z;

        P1_Pixel_Points.push_back({ -Points[i].z, -Points[i].x,  0 });
    }

    return P1_Pixel_Points;
}


//! 添加成像高斯噪声 0均值，低幅值的高斯噪声