#pragma once

#include <string>
#include <mutex>

#include "locator.h"

using namespace std;

/**
 * The BrainData class records the data needed by the Brain during decision-making.
 * Currently, multi-threaded read/write issues are not considered, but this may be addressed in the future if necessary.
 */
/**
*BrainData类记录大脑在决策过程中所需的数据。
* 目前，没有考虑多线程读 / 写问题，但这可能会在未来解决，如果有必要的话。
*/
class BrainData
{
public:
    rclcpp::Time lastSuccessfulLocalizeTime;

    int lastScore = 0;
    int penalty[4];

    /* ------------------------------------ Data Recording ------------------------------------ */

    // Robot position & velocity commands
    Pose2D robotPoseToOdom;  // The robot's Pose in the Odom coordinate system, updated via odomCallback
    Pose2D odomToField;      // The origin of the Odom coordinate system in the Field coordinate system, can be calibrated using known positions, e.g., by calibration at the start of the game
    // 机器人在场地坐标系中当前的位置和方向。
    // 场地中心是原点，x轴指向对方球门（前方），y轴指向左侧。
    // theta的正方向是逆时针
    Pose2D robotPoseToField; // The robot's current position and orientation in the field coordinate system. The field center is the origin, with the x-axis pointing towards the opponent's goal (forward), and the y-axis pointing to the left. The positive direction of theta is counterclockwise.

    // Head position, updated through lowStateCallback
    //当前头部俯仰角，以弧度为单位。0是水平向前，正值是向下
    double headPitch; // The current head pitch, in radians. 0 is horizontal forward, positive is downward.
    // 当前头部偏航角，以弧度为单位。0是向前，正值是向左。
    double headYaw;   // The current head yaw, in radians. 0 is forward, positive is left.

    // Ball
    bool ballDetected = false;    // Whether the camera has detected the ball
    GameObject ball;              // Records the ball's information, including position, bounding box, etc.
    double robotBallAngleToField; // The angle between the robot's vector to the ball and the X-axis in the field coordinate system, (-PI, PI]

    // Other objects on the field
    vector<GameObject> opponents = {}; // Records information about opponent players, including position, bounding box, etc.
    vector<GameObject> goalposts = {}; // Records information about goalposts, including position, bounding box, etc.
    vector<GameObject> markings = {};  // Records information about field markings and intersections

    // Motion planning
    double dribbleTargetAngle;    // The direction for dribbling
    bool dribbleTargetAngleFound; // Whether the dribbling direction planning was successful
    double moveTargetAngle;       // Target direction for movement

    // A collection of utility functions
    vector<FieldMarker> getMarkers();
    // Convert a Pose from the robot coordinate system to the field coordinate system.
    Pose2D robot2field(const Pose2D &poseToRobot);
    // Convert a Pose from the field coordinate system to the robot coordinate system.
    Pose2D field2robot(const Pose2D &poseToField);
};