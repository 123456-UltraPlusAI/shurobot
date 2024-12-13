#include <cmath>

#include "brain.h"
#include "robot_client.h"

#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

#include "booster_msgs/message_utils.h"
//初始化，申明话题发布者
void RobotClient::init()
{
    publisher = brain->create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopicReq", 10);
}

int RobotClient::moveHead(double pitch, double yaw)
{
    //角度上限
    yaw = cap(yaw, brain->config->headYawLimitLeft, brain->config->headYawLimitRight);
    pitch = max(pitch, brain->config->headPitchLimitUp);
    //发布角度
    booster_msgs::msg::RpcReqMsg msg = booster_msgs::CreateRotateHeadMsg(pitch, yaw);
    publisher->publish(msg);
    //返回0，成功执行
    return 0;
}
/*如果doWaveHand为true，则手部动作为kHandOpen，即打开手（挥手）。
如果doWaveHand为false，则手部动作为kHandClose，即闭合手（握拳）。*/
int RobotClient::waveHand(bool doWaveHand)
{
    auto msg = booster_msgs::CreateWaveHandMsg(booster::robot::b1::HandIndex::kRightHand, doWaveHand ? booster::robot::b1::HandAction::kHandOpen : booster::robot::b1::HandAction::kHandClose);
    publisher->publish(msg);
    return 0;
}

int RobotClient::setVelocity(double x, double y, double theta, bool applyMinX, bool applyMinY, bool applyMinTheta)
{
    //日志记录速度
    brain->log->setTimeNow();
    brain->log->log("RobotClient/setVelocity_in",
                    rerun::TextLog(format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta)));
    //创建了三个表示速度向量的集合，分别对应x方向的线速度、y方向的线速度和theta方向的角速度
    rerun::Collection<rerun::Vec2D> vxLine = {{0, 0}, {x, 0}};
    rerun::Collection<rerun::Vec2D> vyLine = {{0, 0}, {0, -y}};
    rerun::Collection<rerun::Vec2D> vthetaLine = {{0, 0}, {2.0 * cos(-theta), 2.0 * sin(-theta)}};
    //记录了机器人的速度向量，包括颜色、半径和绘制顺序
    brain->log->log("robotframe/velocity",
                    rerun::LineStrips2D({vxLine, vyLine, vthetaLine})
                        .with_colors({0xFF0000FF, 0x00FF00FF, 0x0000FFFF})
                        .with_radii({0.05, 0.05, 0.02})
                        .with_draw_order(1.0));

    //对速度进行限制
    double minx = 0.05, miny = 0.08, mintheta = 0.05;
    if (applyMinX && fabs(x) < minx && fabs(x) > 1e-5)
        x = x > 0 ? minx : -minx;
    if (applyMinY && fabs(y) < miny && fabs(y) > 1e-5)
        y = y > 0 ? miny : -miny;
    if (applyMinTheta && fabs(theta) < mintheta && fabs(theta) > 1e-5)
        theta = theta > 0 ? mintheta : -mintheta;
    x = cap(x, brain->config->vxLimit, -brain->config->vxLimit);
    y = cap(y, brain->config->vyLimit, -brain->config->vyLimit);
    theta = cap(theta, brain->config->vthetaLimit, -brain->config->vthetaLimit);
    //记录调整后的速度
    brain->log->log("RobotClient/setVelocity_out",
                    rerun::TextLog(format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta)));
    //发布速度
    auto msg = booster_msgs::CreateMoveMsg(x, y, theta);
    publisher->publish(msg);
    return 0;
}

int RobotClient::moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance)
{
    //生成目标点在机器人坐标系和场地坐标系的变量
    Pose2D target_f, target_r; 
    target_f.x = tx;
    target_f.y = ty;
    target_f.theta = ttheta;
    target_r = brain->data->field2robot(target_f);
    //目标点相对于机器人正前方的角度
    double targetAngle = atan2(target_r.y, target_r.x);
    //机器人到目标点的直线距离
    double targetDist = norm(target_r.x, target_r.y);

    double vx, vy, vtheta;
    //如果机器人已经在目标位置的容忍度范围内，则设置速度为0并返回
    if (
        (fabs(brain->data->robotPoseToField.x - target_f.x) < xTolerance) && (fabs(brain->data->robotPoseToField.y - target_f.y) < yTolerance) && (fabs(toPInPI(brain->data->robotPoseToField.theta - target_f.theta)) < thetaTolerance))
    {
        return setVelocity(0, 0, 0);
    }
    //定义一个静态变量breakOscillate，用于防止机器人在接近目标时的振荡行为。
    static double breakOscillate = 0.0;
    //如果目标距离大于longRangeThreshold减去breakOscillate，
    // 则优先处理角度偏差较大的情况
    if (targetDist > longRangeThreshold - breakOscillate)
    {
        breakOscillate = 0.5;
        //如果目标角度大于turnThreshold，则只设置角速度vtheta并返回
        if (fabs(targetAngle) > turnThreshold)
        {
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
            return setVelocity(0, 0, vtheta);
        }

        // else

        vx = cap(target_r.x, vxLimit, -vxLimit);
        vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
        return setVelocity(vx, 0, vtheta, false, false, false);
    }
    //如果目标距离小于longRangeThreshold，则计算线速度和角速度，并将它们限制在最大速度限制内。
    breakOscillate = 0.0;
    vx = cap(target_r.x, vxLimit, -vxLimit);
    vy = cap(target_r.y, vyLimit, -vyLimit);
    vtheta = cap(target_r.theta, vthetaLimit, -vthetaLimit);
    return setVelocity(vx, vy, vtheta);
}
