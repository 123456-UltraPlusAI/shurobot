#include "brain_log.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"

BrainLog::BrainLog(Brain *argBrain) : enabled(false), brain(argBrain), rerunLog("robocup")
{
    //检查Brain对象的配置中rerunLogEnable是否为false。
    if (!brain->config->rerunLogEnable)
    {
        enabled = false;
        return;
    }
    //connect方法连接到配置中指定的日志服务器地址
    rerun::Error err = rerunLog.connect(brain->config->rerunLogServerAddr);
    if (err.is_err())
    {
        prtErr("Connect rerunLog server failed: " + err.description);
        enabled = false;
        return;
    }
    //如果连接成功，将enabled设置为true，表示日志记录功能已启用
    enabled = true;
}
//如果启用日志，设置时间
void BrainLog::setTimeNow()
{
    if (!enabled)
        return;

    setTimeSeconds(brain->get_clock()->now().seconds());
}

void BrainLog::setTimeSeconds(double seconds)
{
    if (!enabled)
        return;

    rerunLog.set_time_seconds("time", seconds);
}
//将场地和机器人框架的静态几何信息记录到日志中
void BrainLog::logStatics()
{
    if (!enabled)
        return;

    FieldDimensions &fieldDimensions = brain->config->fieldDimensions;
    rerun::Collection<rerun::Vec2D> borders = {{-fieldDimensions.length / 2, -fieldDimensions.width / 2}, {fieldDimensions.length / 2, -fieldDimensions.width / 2}, {fieldDimensions.length / 2, fieldDimensions.width / 2}, {-fieldDimensions.length / 2, fieldDimensions.width / 2}, {-fieldDimensions.length / 2, -fieldDimensions.width / 2}};
    rerun::Collection<rerun::Vec2D> centerLine = {{0, -fieldDimensions.width / 2}, {0, fieldDimensions.width / 2}};
    rerun::Collection<rerun::Vec2D> leftPenalty = {{-fieldDimensions.length / 2, fieldDimensions.penaltyAreaWidth / 2}, {-(fieldDimensions.length / 2 - fieldDimensions.penaltyAreaLength), fieldDimensions.penaltyAreaWidth / 2}, {-(fieldDimensions.length / 2 - fieldDimensions.penaltyAreaLength), -fieldDimensions.penaltyAreaWidth / 2}, {-fieldDimensions.length / 2, -fieldDimensions.penaltyAreaWidth / 2}};
    rerun::Collection<rerun::Vec2D> rightPenalty = {{fieldDimensions.length / 2, fieldDimensions.penaltyAreaWidth / 2}, {(fieldDimensions.length / 2 - fieldDimensions.penaltyAreaLength), fieldDimensions.penaltyAreaWidth / 2}, {(fieldDimensions.length / 2 - fieldDimensions.penaltyAreaLength), -fieldDimensions.penaltyAreaWidth / 2}, {fieldDimensions.length / 2, -fieldDimensions.penaltyAreaWidth / 2}};
    rerun::Collection<rerun::Vec2D> leftGoal = {{-fieldDimensions.length / 2, fieldDimensions.goalAreaWidth / 2}, {-(fieldDimensions.length / 2 - fieldDimensions.goalAreaLength), fieldDimensions.goalAreaWidth / 2}, {-(fieldDimensions.length / 2 - fieldDimensions.goalAreaLength), -fieldDimensions.goalAreaWidth / 2}, {-fieldDimensions.length / 2, -fieldDimensions.goalAreaWidth / 2}};
    rerun::Collection<rerun::Vec2D> rightGoal = {{fieldDimensions.length / 2, fieldDimensions.goalAreaWidth / 2}, {(fieldDimensions.length / 2 - fieldDimensions.goalAreaLength), fieldDimensions.goalAreaWidth / 2}, {(fieldDimensions.length / 2 - fieldDimensions.goalAreaLength), -fieldDimensions.goalAreaWidth / 2}, {fieldDimensions.length / 2, -fieldDimensions.goalAreaWidth / 2}};

    vector<rerun::Vec2D> circle = {{fieldDimensions.circleRadius, 0}};
    for (int i = 0; i < 360; i++)
    {
        double r = fieldDimensions.circleRadius;
        double theta = (i + 1) * M_PI / 180;
        circle.push_back(rerun::Vec2D{r * cos(theta), r * sin(theta)});
    }

    rerunLog.log(
        "field/lines",
        rerun::LineStrips2D({borders, centerLine, leftPenalty, rightPenalty, leftGoal, rightGoal, rerun::Collection<rerun::Vec2D>(circle)})
            .with_colors({0xFFFFFFFF})
            .with_radii({0.01})
            .with_draw_order(0.0));

    rerun::Collection<rerun::Vec2D> xaxis = {{-2, 0}, {2, 0}};
    rerun::Collection<rerun::Vec2D> yaxis = {{0, 2}, {0, -2}};
    rerun::Collection<rerun::Vec2D> border2m = {{-2, -2}, {-2, 2}, {2, 2}, {2, -2}, {-2, -2}};
    rerun::Collection<rerun::Vec2D> border1m = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
    rerun::Collection<rerun::Vec2D> angle = {{3 * cos(0.2), 3 * sin(0.2)}, {0, 0}, {3 * cos(0.2), 3 * sin(-0.2)}};

    rerunLog.log(
        "robotframe/lines",
        rerun::LineStrips2D({xaxis, yaxis, border2m, border1m, angle})
            .with_colors({0xFFFFFFFF})
            .with_radii({0.005, 0.005, 0.005, 0.002, 0.002})
            .with_draw_order(0.0));
}
//确保在日志记录开始之前，时间被正确设置，并且所有必要的静态信息都被记录
void BrainLog::prepare()
{
    if (!enabled)
        return;

    setTimeSeconds(0);
    setTimeNow();
    logStatics();
}
//在屏幕上记录文本信息，通常用于调试或显示日志信息
void BrainLog::logToScreen(string logPath, string text, u_int32_t color, double padding)
{
    if (!enabled)
        return;

    log(
        logPath,
        rerun::Boxes2D::from_mins_and_sizes({{-padding, -padding}}, {{brain->config->camPixX + padding, brain->config->camPixY + padding}})
            .with_labels({text})
            .with_colors({color}));
}