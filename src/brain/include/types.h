#pragma once
#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>
#include <rclcpp/rclcpp.hpp>

using namespace std;

/* ------------------ Struct ------------------------*/

// Pitch dimension information
//存储与足球场尺寸相关的各种测量值
struct FieldDimensions
{
    double length;            // The length of the pitch.
    double width;             // The width of the pitch.
    double penaltyDist;       // The straight-line distance from the penalty spot to the bottom line.
    double goalWidth;         // The width of the goal.
    double circleRadius;      // The radius of the center circle.
    double penaltyAreaLength; // The length of the penalty area.
    double penaltyAreaWidth;  // The width of the penalty area.
    double goalAreaLength;    // The length of the goal area.
    double goalAreaWidth;     // The width of the goal area.
                              // Note: The penalty area is larger than the goal area; the actual lengths and widths of the penalty area and the goal area are smaller. This naming is to be consistent with the competition rules.
};
const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
const FieldDimensions FD_ADULTSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};

// Pose2D, used to record a point on a plane and its orientation
//在二维平面上记录一个点的位置和方向
struct Pose2D
{
    double x = 0;
    double y = 0;
    double theta = 0; // rad, counterclockwise is positive starting from the positive direction of the x-axis.
};

// Point, used to record a three-dimensional point
//三维空间的一个点
struct Point
{
    double x;
    double y;
    double z;
};

// Point2D, used to record a two-dimensional point
struct Point2D
{
    double x;
    double y;
};

// BoundingBox
/*用于表示二维空间中的一个边界框。
边界框通常用于计算机视觉中，定义了一个矩形区域，
可以用于标记图像中的对象、裁剪区域或进行对象检测。*/
struct BoundingBox
{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
};

/// GameObject, used to store the information of important entities in the game, such as Ball, Goalpost, etc. Compared with the detection::DetectedObject in the /detect message, it has more abundant information.
/*string label;：标识对象被识别为何种实体的标签。

BoundingBox boundingBox;：在相机中对象的识别框，以左上角为原点，x轴向右增加，y轴向下增加。

Point2D precisePixelPoint;：对象的精确像素点位置。只有地面地标点才有这个数据。

double confidence;：识别的置信度。

Point posToRobot;：对象在机器人坐标系中的位置。这个位置是二维的，忽略了z值。

string info;：用于存储额外信息的字符串。例如，对于球门柱对象，它可以存储是哪个球门柱。

Point posToField;：对象在场地坐标系中的位置。这个位置是二维的，忽略了z值。x轴向前，y轴向左。

double range;：对象到机器人在场地平面上的投影点的直线距离。

double pitchToRobot, yawToRobot;：对象相对于机器人前方的俯仰角和偏航角，单位为弧度。向下和向左为正。

rclcpp::Time timePoint;：对象被检测到的时间点。*/
struct GameObject
{
    // --- Obtained from the /detect message ---
    string label;              // What the object is identified as.
    BoundingBox boundingBox;   // The recognition box of the object in the camera, with the upper left corner as the origin, x increasing to the right and y increasing downward.
    Point2D precisePixelPoint; // The precise pixel point position of the object. Only ground landmark points have this data.
    double confidence;         // The confidence of the identification.
    Point posToRobot;          // The position of the object in the robot's body coordinate system. The position is 2D, ignoring the z value.

    // --- Calculated and obtained in the processDetectedObject function ---
    string info;                     // Used to store additional information. For example, for a goalpost object, it can store which goalpost it is.
    Point posToField;                // The position of the object in the field coordinate system. The position is 2D, ignoring the z value. x is forward and y is leftward.
    double range;                    // The straight-line distance from the object to the projection point of the robot's center on the field plane.
    double pitchToRobot, yawToRobot; // The pitch and yaw of the object relative to the front of the robot, in rad. Downward and leftward are positive.
    rclcpp::Time timePoint;          // The time when the object was detected.
};

// The numbers corresponding to the Joystick buttons
//游戏手柄按键枚举
enum JoystickBTN
{
    BTN_X,
    BTN_A,
    BTN_B,
    BTN_Y,
    BTN_LB,
    BTN_RB,
    BTN_LT,
    BTN_RT,
    BTN_BACK,
    BTN_START,
};
//摇杆枚举
enum JoystickAX
{
    AX_LX,
    AX_LY,
    AX_RX,
    AX_RY,
    AX_DX,
    AX_DY,
};