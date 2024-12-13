#pragma once
/*RobotClient类。通过调用RobotSDK来控制机器人的所有操作都放在这里*/
#include <iostream>
#include <string>
#include <rerun.hpp>

#include <booster_msgs/msg/rpc_req_msg.hpp>

using namespace std;

class Brain; // Forward declaration

/**
 * The RobotClient class. All operations for controlling the robot by calling the RobotSDK are placed here.
 */
class RobotClient
{
public:
    RobotClient(Brain *argBrain) : brain(argBrain) {}

    void init();

    /**
     * @brief Move the robot's head.
     *
     * @param pitch
     * @param yaw
     *
     * @return int, 0 indicates successful execution.
     */
    int moveHead(double pitch, double yaw);

    /**
     * @brief Set the moving speed of the robot.
     *
     * @param x double, forward (m/s).
     * @param y double, leftward (m/s).
     * @param theta double, counterclockwise rotation angle (rad/s).
     * @param applyMinX, applyMinY, applyMinTheta bool Whether to adjust the command size to prevent non-response when the speed command is too small.
     *
     * @return int, 0 indicates successful execution.
     *
     */
    /*brief 设置机器人的移动速度。
    @param x double，向前的速度（米/秒）。
    @param y double，向左的速度（米/秒）。
    @param theta double，逆时针旋转角度（弧度/秒）。
    @param applyMinX, applyMinY, applyMinTheta bool 是否调整命令大小以防止当速度命令过小时机器人无响应。
    @return int，0表示执行成功。*/
    int setVelocity(double x, double y, double theta, bool applyMinX = true, bool applyMinY = true, bool applyMinTheta = true);

    /**
     * @brief Walk towards a certain Pose in the pitch coordinate system in speed mode. Note that the final orientation should also be reached.
     *
     * @param tx, ty, ttheta double, the target Pose, in the Field coordinate system.
     * @param longRangeThreshold double, when the distance exceeds this value, it is preferred to turn towards the target point and walk over instead of directly adjusting the position.
     * @param turnThreshold double, when the angle difference between the direction of the target point (note that it is not the final orientation ttheta) and the current angle is greater than this value, first turn to face the target.
     * @param vxLimit, vyLimit, vthetaLimit double, the upper limits of speeds in each direction, m/s, rad/s.
     * @param xTolerance, yTolerance, thetaTolerance double, the tolerances for judging that the target point has been reached.
     *
     * @return int The return value of the motion control command, 0 represents success.
     */
    /*@brief 以速度模式走向球场坐标系中的某个特定姿态。注意，最终的朝向也需要达到。
    @param tx, ty, ttheta double，目标姿态，在场地坐标系中。
    @param longRangeThreshold double，当距离超过这个值时，更倾向于先转向目标点然后走过去，而不是直接调整位置。
    @param turnThreshold double，当目标点的方向（注意，这不是最终的朝向 ttheta）与当前角度的差值大于这个值时，先转向以面对目标。
    @param vxLimit, vyLimit, vthetaLimit double，每个方向上速度的上限，单位为米/秒和弧度/秒。
    @param xTolerance, yTolerance, thetaTolerance double，判断目标点已到达时的容忍度。
    @return int 运动控制命令的返回值，0代表成功。*/
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance);

    /**
     * @brief Wave the hand.
     */
    int waveHand(bool doWaveHand);

private:
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
};