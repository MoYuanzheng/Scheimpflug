#include"head.h"

//! 以激光平面与成像平面与镜头平面的交点作为空间直角坐标系原点
//! 激光平面垂直与y轴且过原点
int main() {

    //! 求 光心位置 和 像平面原点
    std::vector<cv::Point3f> point_center = Calculate_Optical_Sensor_Center_Point_2();

    //! 提取 光心位置 
    cv::Point3f Optocal_Center = point_center[0];
    //! 提取 传感器中心
    cv::Point3f Sensor_Center = point_center[1];

    //! 求传感器与激光夹角
    theta_3 = Calculate_Laser_Sensor_Angle();
    theta_9 = PI / 2 - theta_3;
    //! 计算传感器平面方程
    P2 = { 0,0 * cos(theta_9) + (-1) * (-sin(theta_9)),0 * sin(theta_9) + (-1) * cos(theta_9),0 };
    //! 计算P2角点
    Calculate_P2_Corner(Sensor_Center, Sensor_Size, theta_9);

    //! 限制直线方向
    vector<_Line> Line_Direction_Vector_Range = Limit_P2_Intersection_Point_Range(Optocal_Center, P2_4_Corner);

    //! 求随机线面交点
    _Vec_Point_Pair PointPair = Random_Generate_Point_Pair(Optocal_Center, p1, P2, Line_Direction_Vector_Range);

    //! 由P2角点计算P1角点
    std::vector<cv::Point3f> P1_Cornor_Points = Calculate_Special_Points(Optocal_Center, p1, P2_4_Corner);
    std::vector<cv::Point3f> Sensor_Center_Vec = { Sensor_Center };
    std::vector<cv::Point3f> P1_Center_Points_Vec = Calculate_Special_Points(Optocal_Center, p1, Sensor_Center_Vec);


    //! 将固定点对追加到随机点对中 1-4角点 5中心
    PointPair.p1.insert(std::end(PointPair.p1), std::begin(P1_Cornor_Points), std::end(P1_Cornor_Points));
    PointPair.p1.push_back(P1_Center_Points_Vec[0]);

    PointPair.p2.insert(std::end(PointPair.p2), std::begin(P2_4_Corner), std::end(P2_4_Corner));
    PointPair.p2.push_back(Sensor_Center);

    //! 改变像平面坐标系 以像平面角点为原点 建立像素坐标
    std::vector<cv::Point2f>Image_Pixel_Points = Coordinate_System_conversion_to_Pixel_P2(PointPair.p2);

    //! 图像平面叠加畸变模型
    //std::vector<cv::Point3f>Distortion_Point = Simulated_Image_Distortion(Image_Pixel_Points);

    //! 计算最大视场范围
    cv::Size2f FOV = Calculate_FOV_Max(P1_Cornor_Points);

    cv::Point3f P1_Origin = Calculate_Origin(P1_Cornor_Points);

    //! 激光平面（物平面）像素化 
    std::vector<cv::Point3f>P1_Pixel_Points = Coordinate_System_conversion_to_Pixel_P1(PointPair.p1, P1_Origin);

    cout << endl << "p1点 x" << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p1[i].x << ' ';
    }

    cout << endl << "p1点 y" << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p1[i].y << ' ';
    }

    cout << endl << "p1点 z" << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p1[i].z << ' ';
    }


    cout << endl << "p2点 x" << endl;
    for (int i = 0; i < PointPair.p2.size(); i++) {
        cout << PointPair.p2[i].x << ' ';
    }

    cout << endl << "p2点 y" << endl;
    for (int i = 0; i < PointPair.p2.size(); i++) {
        cout << PointPair.p2[i].y << ' ';
    }

    cout << endl << "p2点 z" << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << PointPair.p2[i].z << ' ';
    }


    cout << endl << "平移像素 x" << endl;
    for (int i = 0; i < PointPair.p1.size(); i++) {
        cout << Image_Pixel_Points[i].x << ' ';
    }

    cout << endl << "平移像素 y" << endl;

    for (int i = 0; i < Image_Pixel_Points.size(); i++) {
        cout << Image_Pixel_Points[i].y << ' ';
    }



    // !**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--**--
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
    err_first = cv::calibrateCamera(obj, img, Image_Size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, cv::CALIB_RATIONAL_MODEL);

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


//! 求取光心及传感器中心坐标
std::vector<cv::Point3f> Calculate_Optical_Sensor_Center_Point_2() {

    //! P[0]为光心 P[1]为像原点
    std::vector<cv::Point3f> P;

    //! 光心的坐标为（0, -a, -e）
    //! 传感器中心为(0,-(a+f),-(e-k) )
    //! 镜头光心坐标
    cv::Point3f Optical_Center = { 0.0, -static_cast<float>(distance_a), -static_cast<float>(distance_e) };

    P.push_back(Optical_Center);

    cv::Point3f Sensor_Center = { 0.0, -static_cast<float>((distance_a + distance_f)) ,-static_cast<float>(distance_h) };

    P.push_back(Sensor_Center);

    return P;
}

//! 计算传感器与激光平面夹角 theta_9
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-TnuodwWuaosALTxEhk1cvQL7nIe
double Calculate_Laser_Sensor_Angle() {

    return PI / 2 - acos((distance_a + distance_b * cos(theta_1)) / sqrt(pow((distance_a - distance_b / (sqrt(2))), 2) + pow((distance_a + distance_b / (sqrt(2))), 2)));

}

//! 判断 P1 P2 相交
bool Is_Intersect(_Plane P1, _Plane P2, _Line L) {
    if (P1.A * L.a + P1.B * L.b + P1.C * L.c == 0 || P2.A * L.a + P2.B * L.b + P2.C * L.c == 0) {
        return false;
    }
    return true;
}

//! 限制：线方向向量（靶面出发）
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-E7sudYxHNoGZzYxQfIlcWUI0nXc
std::vector<_Line> Limit_P2_Intersection_Point_Range(cv::Point3f Optocal_Center, vector<cv::Point3f> P2_4_Corner) {

    //! 得直线方程
    vector<_Line> Line_Direction_Vector_Range;

    for (int i = 0; i < P2_4_Corner.size(); i++) {
        Line_Direction_Vector_Range.push_back({
            (Optocal_Center.x - P2_4_Corner[i].x),
            (Optocal_Center.y - P2_4_Corner[i].y),
            (Optocal_Center.z - P2_4_Corner[i].z)
            });
    }
    return Line_Direction_Vector_Range;
}



//! 随机生成穿光心的直线 返回该直线与两个面交点
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-YG9tdhCGmoYaQUx1pHTcHaMnnk9
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

//! 两平面特殊点 点对生成
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-EC6udrjTPoNfAMxyf4CctmKcnfg
std::vector<cv::Point3f> Calculate_Special_Points(cv::Point3f optical_center, _Plane plane, vector<cv::Point3f> Special_Points) {
    std::vector<cv::Point3f> Another_Points;
    //std::vector<_Line> Line_Special;

    for (int i = 0; i < Special_Points.size(); i++) {

        //Line_Special.push_back({ Special_Points[i].x - optical_center.x,Special_Points[i].y - optical_center.y,Special_Points[i].z - optical_center.z });
        //Another_Points.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, plane, Line_Special[i]));
        Another_Points.push_back(
            Calculate_Line_Plane_Intersection_Point(
                optical_center, plane, {
                    Special_Points[i].x - optical_center.x,
                    Special_Points[i].y - optical_center.y,
                    Special_Points[i].z - optical_center.z
                }
        ));
    }

    //_Line Line_LU = { Special_Points[0].x - optical_center.x,Special_Points[0].y - optical_center.y,Special_Points[0].z - optical_center.z };
    //_Line Line_RU = { Special_Points[1].x - optical_center.x,Special_Points[1].y - optical_center.y,Special_Points[1].z - optical_center.z };
    //_Line Line_LD = { Special_Points[2].x - optical_center.x,Special_Points[2].y - optical_center.y,Special_Points[2].z - optical_center.z };
    //_Line Line_RD = { Special_Points[3].x - optical_center.x,Special_Points[3].y - optical_center.y,Special_Points[3].z - optical_center.z };

    //Another_Points.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, plane, Line_LU));
    //Another_Points.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, plane, Line_RU));
    //Another_Points.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, plane, Line_LD));
    //Another_Points.push_back(Calculate_Line_Plane_Intersection_Point(optical_center, plane, Line_RD));

    return Another_Points;
}

//! 将传感器平面转为以角点建立坐标系（旋转、平移及像素化）
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-BvkTdi3knoIT1nxfleAclVnlnDc
std::vector<cv::Point2f> Coordinate_System_conversion_to_Pixel_P2(vector<cv::Point3f> points) {

    //! 像平面坐标由世界坐标系转为传感器（像平面）坐标系，以橡心为原点
    std::vector<cv::Point3f> points_R;
    for (int i = 0; i < points.size(); i++) {

        points_R.push_back({ points[i].x,0,0 });

        //! 旋转 交点逆时针旋转 PI/2 - theta_9 个弧度
        points_R[i].y = points[i].y * cos(theta_9) + points[i].z * sin(theta_9);
        points_R[i].z = -points[i].y * sin(theta_9) + points[i].z * cos(theta_9);

    }

    cout << "旋转 x" << endl;
    for (int i = 0; i < points_R.size(); i++) {
        cout << points_R[i].x << ' ';
    }

    cout << "旋转 y" << endl;
    for (int i = 0; i < points_R.size(); i++) {
        cout << points_R[i].y << ' ';
    }


    //! 平移至角点 建立像素坐标系
    std::vector<cv::Point3f> points_RT;
    for (int i = 0; i < points_R.size(); i++) {

        points_RT.push_back({
            (points_R[i].y - points_R[LINE_NUMBER + 1].y),
           -(points_R[i].x - points_R[LINE_NUMBER + 1].x),
            points_R[i].z }
        );
    }

    //! 转像素  按照采样比进行像素转换
    vector<cv::Point2f> Image_Pixel_Points;
    for (int i = 0; i < points_RT.size(); i++) {
        Image_Pixel_Points.push_back({
            points_RT[i].x * static_cast<float>(Cols_Sample_Rate) ,
            points_RT[i].y * static_cast<float>(Rows_Sample_Rate)
            }
        );
    }



    return Image_Pixel_Points;
}

//! 添加成像平面p2径向畸变
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-RrKrdnq6foTnvoxoDsIcK1Fan7g
//std::vector<cv::Point3f> Simulated_Image_Distortion(vector<cv::Point3f> points) {
//
//    for (int i = 0; i < points.size(); i++) {
//        //! 计算距离
//        //! 转到相机坐标系下（由于当前是以图像中心为坐标原点，故不需要减去cx cy）
//        points[i].x = points[i].x / fx;
//        points[i].y = points[i].y / fy;
//        float Distance_Center = sqrt(pow(points[i].x, 2) + pow(points[i].y, 2));
//        //! 1 + xxx 为桶形失真，  1 - xxx 为枕形失真
//        float lambda = 1 + k1 * pow(Distance_Center, 2 * 1) + k2 * pow(Distance_Center, 2 * 2) + k3 * pow(Distance_Center, 2 * 3);
//        points[i].x = points[i].x * lambda * fx;
//        points[i].y = points[i].y * lambda * fy;
//    }
//    //! 返回添加完畸变的vector坐标
//    return points;
//}

//! P1平面像素化
//! https://p3cw4jny8c.feishu.cn/docx/DlCGdkHJ3oIHvNxt5YsczCGVnIb#part-JsncdLzI0ojWqNxalL9c8UJjnIf
std::vector<cv::Point3f> Coordinate_System_conversion_to_Pixel_P1(std::vector<cv::Point3f> Points, cv::Point3f Origin) {
    std::vector<cv::Point3f> P1_Pixel_Points;

    for (int i = 0; i < Points.size(); i++) {

        //! 平移
        Points[i].x = Points[i].x - Origin.x;
        Points[i].y = Points[i].y - Origin.y;
        Points[i].z = Points[i].z - Origin.z;

        //! 交换并压栈
        P1_Pixel_Points.push_back({ -Points[i].z, -Points[i].x,  0 });
        //P1_Pixel_Points.push_back({ Points[i].x,   0 ,Points[i].z });
    }

    //cout << endl << "p1平移像素 x" << endl;
    //for (int i = 0; i < P1_Pixel_Points.size(); i++) {
    //    cout << P1_Pixel_Points[i].x << ' ';
    //}

    //cout << endl << "p1平移像素 y" << endl;

    //for (int i = 0; i < P1_Pixel_Points.size(); i++) {
    //    cout << P1_Pixel_Points[i].y << ' ';
    //}

    return P1_Pixel_Points;
}

//! 计算 FOV 尺寸范围
cv::Size2f Calculate_FOV_Max(std::vector<cv::Point3f> Cornor_Points) {
    //! 靶面的 LD->index=2 对应激光平面 RU->index=1
    double width = Cornor_Points[2].x - Cornor_Points[3].x;//LU.X0-RU.X1
    double height = Cornor_Points[2].z - Cornor_Points[1].z;//ld.y-ru.y
    return { width, height };
}

//! 计算像素原点
cv::Point3f Calculate_Origin(std::vector<cv::Point3f> P1_Cornor_Points) {
    //! 取最大的 Z 最小的 X.
    float X = P1_Cornor_Points[0].x;
    float Z = P1_Cornor_Points[0].z;

    for (int i = 1; i < P1_Cornor_Points.size(); i++) {
        if (P1_Cornor_Points[i].x < X && P1_Cornor_Points[i].x >= 0) {
            X = P1_Cornor_Points[i].x;
        }

        if (P1_Cornor_Points[i].z > Z) {
            Z = P1_Cornor_Points[i].z;
        }
    }

    return { X,0,Z };
}

//! 计算P2角点
void Calculate_P2_Corner(cv::Point3f Sensor_Center, cv::Size2f Sensor_Size, double theta_9) {
    P2_4_Corner.push_back({
        -Sensor_Size.height / 2,
        Sensor_Center.y + static_cast<float>(Sensor_Size.width / 2 * cos(theta_9)),
        Sensor_Center.z + static_cast<float>(Sensor_Size.width / 2 * sin(theta_9))
        });
    P2_4_Corner.push_back({
        +Sensor_Size.height / 2,
       Sensor_Center.y + static_cast<float>(Sensor_Size.width / 2 * cos(theta_9)),
       Sensor_Center.z + static_cast<float>(Sensor_Size.width / 2 * sin(theta_9))
        });
    P2_4_Corner.push_back({
        -Sensor_Size.height / 2,
       Sensor_Center.y - static_cast<float>(Sensor_Size.width / 2 * cos(theta_9)),
       Sensor_Center.z - static_cast<float>(Sensor_Size.width / 2 * sin(theta_9))
        });
    P2_4_Corner.push_back({
        +Sensor_Size.height / 2,
        Sensor_Center.y - static_cast<float>(Sensor_Size.width / 2 * cos(theta_9)),
        Sensor_Center.z - static_cast<float>(Sensor_Size.width / 2 * sin(theta_9))
        });
}