#pragma once
/*RobotClient�ࡣͨ������RobotSDK�����ƻ����˵����в�������������*/
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
    /*brief ���û����˵��ƶ��ٶȡ�
    @param x double����ǰ���ٶȣ���/�룩��
    @param y double��������ٶȣ���/�룩��
    @param theta double����ʱ����ת�Ƕȣ�����/�룩��
    @param applyMinX, applyMinY, applyMinTheta bool �Ƿ���������С�Է�ֹ���ٶ������Сʱ����������Ӧ��
    @return int��0��ʾִ�гɹ���*/
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
    /*@brief ���ٶ�ģʽ����������ϵ�е�ĳ���ض���̬��ע�⣬���յĳ���Ҳ��Ҫ�ﵽ��
    @param tx, ty, ttheta double��Ŀ����̬���ڳ�������ϵ�С�
    @param longRangeThreshold double�������볬�����ֵʱ������������ת��Ŀ���Ȼ���߹�ȥ��������ֱ�ӵ���λ�á�
    @param turnThreshold double����Ŀ���ķ���ע�⣬�ⲻ�����յĳ��� ttheta���뵱ǰ�ǶȵĲ�ֵ�������ֵʱ����ת�������Ŀ�ꡣ
    @param vxLimit, vyLimit, vthetaLimit double��ÿ���������ٶȵ����ޣ���λΪ��/��ͻ���/�롣
    @param xTolerance, yTolerance, thetaTolerance double���ж�Ŀ����ѵ���ʱ�����̶ȡ�
    @return int �˶���������ķ���ֵ��0����ɹ���*/
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance);

    /**
     * @brief Wave the hand.
     */
    int waveHand(bool doWaveHand);

private:
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
};