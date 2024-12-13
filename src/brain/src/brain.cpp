#include <iostream>
#include <string>

#include "brain.h"
#include "utils/print.h"
#include "utils/math.h"

using namespace std;
using std::placeholders::_1;

Brain::Brain() : rclcpp::Node("brain_node")
{
    // Note that the parameters must be declared here first, otherwise they cannot be read in the program either.
    //declare_parameter申明节点特有的参数
    declare_parameter<int>("game.team_id", 0);
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.player_start_pos", "");

    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.95);
    declare_parameter<double>("robot.yaw_offset", 0.1);

    declare_parameter<bool>("rerunLog.enable", false);
    declare_parameter<string>("rerunLog.server_addr", "");
    declare_parameter<int>("rerunLog.img_interval", 10);

    // The tree_file_path is configured in launch.py and not placed in config.yaml.
    declare_parameter<string>("tree_file_path", "");
    /*这些参数被声明后，可以通过ROS2的参数服务器进行配置，
    也可以在启动节点时通过命令行进行覆盖。
    这些参数通常用于配置节点的行为，例如设置球队ID、玩家ID、机器人的高度、里程计因子等。
    通过在构造函数中声明这些参数，Brain 类可以在其方法中访问和使用这些参数。*/
}


/*这个方法初始化了 Brain 类的所有主要组件，
并设置了各种ROS2主题的订阅者，
以便节点可以接收和处理来自不同传感器和接口的数据。
这些订阅者将允许 Brain 节点与ROS2环境中的其他节点进行通信和交互。*/
void Brain::init()
{
    // Make sure to load the configuration first, and then the config can be used.
    config = std::make_shared<BrainConfig>();
    loadConfig();//调用 loadConfig 方法，从配置文件或其他来源加载配置。

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();

    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);

    locator->init(config->fieldDimensions, 4, 0.5);

    tree->init();//调用 BrainTree 实例的 init 方法，初始化决策树。

    client->init();//调用 RobotClient 实例的 init 方法，初始化机器人客户端。

    log->prepare();//调用 BrainLog 实例的 prepare 方法，准备日志记录。

    data->lastSuccessfulLocalizeTime = get_clock()->now();//设置最后成功定位的时间戳。

    //创建一个订阅者，订阅 "/joy" 主题，用于接收游戏手柄的输入。
    joySubscription = create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bind(&Brain::joystickCallback, this, _1));
    //创建一个订阅者，订阅 "/robocup/game_controller" 主题，用于接收裁判机控制数据。
    gameControlSubscription = create_subscription<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 1, bind(&Brain::gameControlCallback, this, _1));
    //创建一个订阅者，订阅 "/booster_vision/detection" 主题，用于接收视觉检测数据。
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", 1, bind(&Brain::detectionsCallback, this, _1));
    //创建一个订阅者，订阅 "/odometer_state" 主题，用于接收里程计状态数据。
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state", 1, bind(&Brain::odometerCallback, this, _1));
    //创建一个订阅者，订阅 "/low_state" 主题，用于接收机器人底层状态数据。
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", 1, bind(&Brain::lowStateCallback, this, _1));
    //创建一个订阅者，订阅 "/camera/camera/color/image_raw" 主题，用于接收原始图像数据。
    imageSubscription = create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 1, bind(&Brain::imageCallback, this, _1));
    //创建一个订阅者，订阅 "/head_pose" 主题，用于接收头部姿态数据。
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose", 1, bind(&Brain::headPoseCallback, this, _1));
}


/*这段代码是 Brain 类的 loadConfig 方法的实现，
它负责从ROS2的参数服务器获取配置参数，
并存储到 BrainConfig 类的实例中。*/
void Brain::loadConfig()
{
    //从参数服务器查找信息，存储到config对象中的对应属性
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.player_start_pos", config->playerStartPos);

    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("robot.odom_factor", config->robotOdomFactor);
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);

    get_parameter("rerunLog.enable", config->rerunLogEnable);
    get_parameter("rerunLog.server_addr", config->rerunLogServerAddr);
    get_parameter("rerunLog.img_interval", config->rerunLogImgInterval);

    get_parameter("tree_file_path", config->treeFilePath);

    // handle the parameters
    config->handle();//调用 BrainConfig 实例的 handle 方法，处理获取到的参数。

    // debug after handle the parameters
    ostringstream oss;//创建一个字符串流 oss，用于构建调试信息。
    config->print(oss);//调用 BrainConfig 实例的 print 方法，将配置信息打印到字符串流 oss 中。
    prtDebug(oss.str());//调用 prtDebug 方法，将字符串流中的配置信息作为调试信息输出。
}

/**
 * will be called in the Ros2 loop
 */
/*tick 方法通常被设计为在ROS2的执行器（executor）循环中定期执行，
以确保节点能够响应环境变化和执行任务。
在这种情况下，tick 方法通过更新记忆和决策树来驱动 Brain 节点的行为。
这种方法允许节点在每个时间步更新其状态，并根据最新的信息做出决策。*/
void Brain::tick()
{
    /*调用 Brain 类的 updateMemory 方法，
    这个方法可能负责更新节点的内部状态或记忆，
    例如，更新传感器数据、机器人状态或其他相关信息。*/
    updateMemory();
    /*调用 BrainTree 类实例的 tick 方法，
    这个方法可能负责处理决策逻辑，
    根据当前的机器人状态和环境信息来决定下一步行动。*/
    tree->tick();
}

/*这个方法的主要功能是更新与球相关的内存信息，
并根据当前的游戏状态和球的位置来决定是否需要等待对手开球。*/
void Brain::updateMemory()
{
    updateBallMemory();//更新与球相关的内存信息，例如球的位置。

    static Point ballPos;//球坐标
    static rclcpp::Time kickOffTime;//存储踢球开始的时间
    /*检查当前角色是否为“striker”（前锋），
    并且游戏状态为“SET”（设置）且不是我们的开球方，
    或者替补状态为“SET”且不是我们的替补开球方。*/
    if (
        tree->getEntry<string>("player_role") == "striker" && ((tree->getEntry<string>("gc_game_state") == "SET" && !tree->getEntry<bool>("gc_is_kickoff_side")) || (tree->getEntry<string>("gc_game_sub_state") == "SET" && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side"))))
    {
        ballPos = data->ball.posToRobot;//更新 ballPos 为球相对于机器人的位置
        kickOffTime = get_clock()->now();//记录当前时间为开球时间
        //在决策树中设置一个条目，表示我们正在等待对手开球。
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    }
    //如果之前已经设置了等待对手开球的状态。
    else if (tree->getEntry<bool>("wait_for_opponent_kickoff"))
    {
        //检查球的位置是否发生了显著变化（超过0.3米），或者时间已经过去了10秒
        if (
            norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y) > 0.3 || (get_clock()->now() - kickOffTime).seconds() > 10.0)
        {
            //不再等待对手开球，更新决策树中的相应条目。
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }
}
/*主要功能是更新球的位置信息，
并根据球的位置和机器人的当前状态来更新决策树和日志*/
void Brain::updateBallMemory()
{
    // update Pose to field from Pose to robot (based on odom)
    //存储从机器人坐标系到场地坐标系的转换参数
    double xfr, yfr, thetafr; // fr = field to robot
    /*计算 yfr, xfr, thetafr：
    使用三角函数和机器人相对于场地的姿态（data->robotPoseToField），
    计算球的位置从机器人坐标系到场地坐标系的转换参数。*/
    yfr = sin(data->robotPoseToField.theta) * data->robotPoseToField.x - cos(data->robotPoseToField.theta) * data->robotPoseToField.y;
    xfr = -cos(data->robotPoseToField.theta) * data->robotPoseToField.x - sin(data->robotPoseToField.theta) * data->robotPoseToField.y;
    thetafr = -data->robotPoseToField.theta;
    //将球的位置从场地坐标系转换到机器人坐标系
    transCoord(
        data->ball.posToField.x, data->ball.posToField.y, 0,
        xfr, yfr, thetafr,
        data->ball.posToRobot.x, data->ball.posToRobot.y, data->ball.posToRobot.z);
    //计算球到机器人的距离，并将其存储在 data->ball.range 中。
    data->ball.range = sqrt(data->ball.posToRobot.x * data->ball.posToRobot.x + data->ball.posToRobot.y * data->ball.posToRobot.y);
    //在决策树中设置球的距离条目
    tree->setEntry<double>("ball_range", data->ball.range);
    //计算球相对于机器人的偏航角和俯仰角
    data->ball.yawToRobot = atan2(data->ball.posToRobot.y, data->ball.posToRobot.x);
    data->ball.pitchToRobot = asin(config->robotHeight / data->ball.range);

    // mark ball as lost if long time no see
    /*检查球是否丢失：如果自上次看到球以来的时间超过了配置的 memoryLength，
    则将球的位置已知条目设置为 false，并标记球为丢失。*/
    if (get_clock()->now().seconds() - data->ball.timePoint.seconds() > config->memoryLength)
    {
        tree->setEntry<bool>("ball_location_known", false);
        data->ballDetected = false;
    }

    // log mem ball pos
    //设置日志的当前时间
    log->setTimeNow();
    /*记录球的位置到日志中，
    使用 rerun::LineStrips2D 来表示球的位置，
    并根据球的位置是否已知来设置颜色*/
    log->log("field/memball",
             rerun::LineStrips2D({
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x - 0.2, -data->ball.posToField.y}, {data->ball.posToField.x + 0.2, -data->ball.posToField.y}},
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x, -data->ball.posToField.y - 0.2}, {data->ball.posToField.x, -data->ball.posToField.y + 0.2}},
                                 })
                 .with_colors({tree->getEntry<bool>("ball_location_known") ? 0xFFFFFFFF : 0xFF0000FF})
                 .with_radii({0.005})
                 .with_draw_order(30));
}
/*确定球到对方球门柱的相对角度，决定如何瞄准射门以及如何调整其射门策略
margin 参数可以用来考虑机器人射门时的误差范围或球的移动空间。*/
vector<double> Brain::getGoalPostAngles(const double margin)
{
    //存储对方球门柱的坐标
    double leftX, leftY, rightX, rightY;
    //根据场地的尺寸设置对方球门柱的默认坐标
    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;

    /*更新球门柱坐标：如果找到对方左侧球门柱（"oppo-left"），
    则更新 leftX 和 leftY；如果找到对方右侧球门柱（"oppo-right"），
    则更新 rightX 和 rightY*/
    for (int i = 0; i < data->goalposts.size(); i++)
    {
        auto post = data->goalposts[i];
        if (post.info == "oppo-left")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.info == "oppo-right")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }
    /*计算角度 theta_l 和 theta_r：
    使用 atan2 函数计算从球的位置到左侧和右侧球门柱的角度，
    同时考虑传入的 margin 参数。*/
    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);
    //创建一个包含两个角度的 vector<double> 并返回。
    vector<double> vec = {theta_l, theta_r};
    return vec;
}
/*校准和更新机器人、球和对手的位置信息，
确保所有的位置数据都是基于最新的场地坐标系。
这对于机器人的导航、定位和决策至关重要，
因为它依赖于准确的场地坐标来执行任务。*/
//x,y,theat代表机器人在场地坐标系下的x坐标、y坐标和姿态。
void Brain::calibrateOdom(double x, double y, double theta)
{
    //存储从里程计坐标系到机器人坐标系的转换参数
    double x_or, y_or, theta_or; // or = odom to robot
    //计算里程计的位置从机器人坐标系到里程计坐标系的转换参数
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;
    //将里程计的位置从机器人坐标系转换到场地坐标系。
    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);
    //将机器人的位置从里程计坐标系转换到场地坐标系。
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
    //临时存储不关心的z坐标或姿态值
    double placeHolder;
    // ball
    //将球的位置从机器人坐标系转换到场地坐标系
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder);

    // opponents
    //将对手的位置从机器人坐标系转换到场地坐标系
    for (int i = 0; i < data->opponents.size(); i++)
    {
        auto obj = data->opponents[i];
        transCoord(
            obj.posToRobot.x, obj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            obj.posToField.x, obj.posToField.y, placeHolder);
    }
}
/*获取从某个特定时间点到现在经过的时间（以毫秒为单位）
检测自上次检测到某个事件以来是否超过了特定的时间阈值*/
double Brain::msecsSince(rclcpp::Time time)
{
    return (this->get_clock()->now() - time).nanoseconds() / 1e6;
}
//接收遥控器（部分）
void Brain::joystickCallback(const sensor_msgs::msg::Joy &msg)
{
    //检查是否没有按下任何肩键（LT, RT, LB, RB）
    if (msg.buttons[BTN_LT] == 0 && msg.buttons[BTN_RT] == 0 && msg.buttons[BTN_LB] == 0 && msg.buttons[BTN_RB] == 0)
    {
        //检查按钮 B 是否被按下
        if (msg.buttons[BTN_B] == 1)
        {
            //如果按钮 B 被按下，在决策树中设置 B_pressed 条目为 true。
            tree->setEntry<bool>("B_pressed", true);
            prtDebug("B is pressed");
        }
        else if (msg.buttons[BTN_B] == 0)
        {
            //如果按钮 B 被释放，在决策树中设置 B_pressed 条目为 false。
            tree->setEntry<bool>("B_pressed", false);
            prtDebug("B is released");
        }
    }
    //检查左肩键（LT）是否被按下
    else if (msg.buttons[BTN_LT] == 1)
    {
        //检查右侧摇杆的 X 轴或 Y 轴是否有输入
        if (msg.axes[AX_DX] || msg.axes[AX_DY])
        {
            //右摇杆的 X 轴输入调整速度因子 vxFactor。
            config->vxFactor += 0.01 * msg.axes[AX_DX];
            //根据右摇杆的 Y 轴输入调整偏航偏移 yawOffset。
            config->yawOffset += 0.01 * msg.axes[AX_DY];
            prtDebug(format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset));
        }
        //检查按钮 X 是否被按下
        if (msg.buttons[BTN_X] == 1)
        {
            //在决策树中设置控制状态为 1
            tree->setEntry<int>("control_state", 1);
            //发送命令使机器人停止移动
            client->setVelocity(0., 0., 0.);
            发送命令使机器人头部归位
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
        }
        else if (msg.buttons[BTN_A] == 1)
        {
            //在决策树中设置控制状态为 2
            tree->setEntry<int>("control_state", 2);
            //在决策树中设置里程计校准状态为未校准。
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
        }
        else if (msg.buttons[BTN_B] == 1)
        {
            //在决策树中设置控制状态为 3
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
        }
        else if (msg.buttons[BTN_Y] == 1)
        {
            //切换角色：如果当前角色是“striker”，
            // 则切换为“goal_keeper”，反之亦然
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
        }
    }
}
/*处理从裁判机接收到的消息，
并根据消息更新决策树中的游戏状态、子状态、点球信息和得分信息*/
void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    //获取最后的游戏状态：从决策树中获取最后的 gc_game_state
    auto lastGameState = tree->getEntry<string>("gc_game_state");
    //定义游戏状态映射：定义一个包含所有可能游戏状态的向量 gameStateMap
    vector<string> gameStateMap = {
        "INITIAL", // Initialization state, players are ready outside the field.
        "READY",   // Ready state, players enter the field and walk to their starting positions.
        "SET",     // Stop action, waiting for the referee machine to issue the instruction to start the game.
        "PLAY",    // Normal game.
        "END"      // The game is over.
    };
    //获取当前的游戏状态，并将其存储在 gameState 变量中。
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    //更新决策树中的游戏状态：在决策树中设置当前的游戏状态
    tree->setEntry<string>("gc_game_state", gameState);
    /*根据消息中的 kick_off_team 字段和配置中的 teamId，
    判断当前队伍是否为开球方，并在决策树中设置相应的条目。*/
    bool isKickOffSide = (msg.kick_off_team == config->teamId);
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);
    /*获取游戏子状态类型：根据消息中的 secondary_state 字段，
    判断游戏子状态类型，并在决策树中设置相应的条目。
    定义游戏子状态映射：定义一个包含所有可能游戏子状态的向量 gameSubStateMap。*/
    string gameSubStateType = static_cast<int>(msg.secondary_state) == 0 ? "NONE" : "FREE_KICK";
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};
    //获取当前游戏子状态
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    //是否为子状态的开球方
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == config->teamId);
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);
    /*获取本队信息：根据配置中的 teamId，
    从消息中的 teams 字段中获取本队的信息，
    并将其存储在 myTeamInfo 变量中*/
    game_controller_interface::msg::TeamInfo myTeamInfo;
    if (msg.teams[0].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[0];
    }
    else if (msg.teams[1].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[1];
    }
    else
    {

        prtErr("received invalid game controller message");
        return;
    }
    /*更新点球信息：根据本队信息，更新 data->penalty 数组，
    并在决策树中设置是否处于点球状态*/
    data->penalty[0] = static_cast<int>(myTeamInfo.players[0].penalty);
    data->penalty[1] = static_cast<int>(myTeamInfo.players[1].penalty);
    data->penalty[2] = static_cast<int>(myTeamInfo.players[2].penalty);
    data->penalty[3] = static_cast<int>(myTeamInfo.players[3].penalty);
    double isUnderPenalty = (data->penalty[config->playerId] != 0);
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);
    //更新得分信息：根据本队信息，更新得分，并在决策树中设置是否刚刚得分
    int curScore = static_cast<int>(myTeamInfo.score);
    if (curScore > data->lastScore)
    {
        tree->setEntry<bool>("we_just_scored", true);
        data->lastScore = curScore;
    }
    //重置刚刚得分的状态：如果游戏状态为 "SET"，则在决策树中重置刚刚得分的状态。
    if (gameState == "SET")
    {
        tree->setEntry<bool>("we_just_scored", false);
    }
}
/*处理从视觉系统接收到的检测消息，将检测到的游戏对象分类，
并记录相关的日志信息*/
void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    //从消息中提取游戏对象。
    auto gameObjects = getGameObjects(msg);

    //定义各种游戏对象的向量：定义用于存储球、球门柱、人、机器人、障碍物和标记的向量
    vector<GameObject> balls, goalPosts, persons, robots, obstacles, markings;
    //遍历游戏对象：遍历 gameObjects 向量，根据对象的标签将它们分类到相应的向量中
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalPosts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            if (tree->getEntry<bool>("treat_person_as_robot"))
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }
    //处理检测到的球。
    detectProcessBalls(balls);
    //处理检测到的标记。
    detectProcessMarkings(markings);
    //检查日志是否启用：如果日志未启用，则返回
    if (!log->isEnabled())
        return;

    // log detection boxes to rerun
    //获取当前时间：获取当前时间，用于日志记录
    auto detection_time_stamp = msg.header.stamp;
    rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto now = get_clock()->now();
    //定义检测颜色映射：定义一个映射，将不同的标签映射到不同的颜色
    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
    };

    // for logging boundingBoxes
    //定义用于日志记录的向量：定义用于存储最小点、大小、标签和颜色的向量
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame
    //遍历 gameObjects 向量，为每个对象生成日志记录所需的数据
    for (int i = 0; i < gameObjects.size(); i++)
    {
        auto obj = gameObjects[i];
        auto label = obj.label;
        labels.push_back(rerun::Text(format("%s x:%.2f y:%.2f c:%.2f", obj.label.c_str(), obj.posToRobot.x, obj.posToRobot.y, obj.confidence)));
        points.push_back(rerun::Vec2D{obj.posToField.x, -obj.posToField.y});
        points_r.push_back(rerun::Vec2D{obj.posToRobot.x, -obj.posToRobot.y});
        mins.push_back(rerun::Vec2D{obj.boundingBox.xmin, obj.boundingBox.ymin});
        sizes.push_back(rerun::Vec2D{obj.boundingBox.xmax - obj.boundingBox.xmin, obj.boundingBox.ymax - obj.boundingBox.ymin});

        auto it = detectColorMap.find(label);
        if (it != detectColorMap.end())
        {
            colors.push_back(detectColorMap[label]);
        }
        else
        {
            colors.push_back(rerun::Color(0xFFFFFFFF));
        }
    }

    double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    log->setTimeSeconds(time);
    log->log("image/detection_boxes",
             rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                 .with_labels(labels)
                 .with_colors(colors));
    //记录场地上的检测点：使用 rerun::Points2D 类记录场地上的检测点。
    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
             // .with_labels(labels)
    );
    //记录场地上的检测点：使用 rerun::Points2D 类记录场地上的检测点。
    log->log("robotframe/detection_points",
             rerun::Points2D(points_r)
                 .with_colors(colors)
             // .with_labels(labels)
    );
}
/*处理从里程计接收到的位置信息，将其转换为场地坐标系下的位置，
并记录到日志中，以便后续的分析和可视化。
通过这种方式，Brain 节点能够根据实时的里程计数据更新其对机器人位置的理解。*/
void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{
    /*更新机器人相对于里程计的位置：
    将接收到的里程计消息中的 x、y 和 theta 值乘以配置中的 robotOdomFactor*/
    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;
    //将机器人的位置从里程计坐标系转换到场地坐标系
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
    /*记录机器人在场地上的位置。使用 rerun::Points2D 类来记录两个点：一个是机器人的中心位置，另一个是机器人的一个方向指示点（通常是机器人的前端）。
方向指示点的计算：使用机器人的姿态 theta 计算方向指示点的位置，这个点位于机器人中心位置前方0.1米处，方向沿着机器人的前进方向。
设置点的半径：为两个点设置半径，中心点半径为0.2米，方向指示点半径为0.1米。
设置点的颜色：为两个点设置颜色，中心点颜色为紫色（0xFF6666FF），方向指示点颜色为红色（0xFF0000FF）。*/
    log->setTimeNow();
    log->log("field/robot",
             rerun::Points2D({{data->robotPoseToField.x, -data->robotPoseToField.y}, {data->robotPoseToField.x + 0.1 * cos(data->robotPoseToField.theta), -data->robotPoseToField.y - 0.1 * sin(data->robotPoseToField.theta)}})
                 .with_radii({0.2, 0.1})
                 .with_colors({0xFF6666FF, 0xFF0000FF}));
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    /*更新机器人头部的姿态：
    从消息中提取头部偏航（headYaw）和俯仰（headPitch）的角度*/
    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;

    log->setTimeNow();
    /*记录IMU（惯性测量单元）的状态：使用 log->log 方法记录IMU的状态：
    姿态角（Roll, Pitch, Yaw）：记录IMU的三个姿态角。
    加速度（Acc）：记录IMU的三个加速度分量。
    陀螺仪（Gyro）：记录IMU的三个角速度分量。*/
    log->log("low_state_callback/imu/rpy/roll", rerun::Scalar(msg.imu_state.rpy[0]));
    log->log("low_state_callback/imu/rpy/pitch", rerun::Scalar(msg.imu_state.rpy[1]));
    log->log("low_state_callback/imu/rpy/yaw", rerun::Scalar(msg.imu_state.rpy[2]));
    log->log("low_state_callback/imu/acc/x", rerun::Scalar(msg.imu_state.acc[0]));
    log->log("low_state_callback/imu/acc/y", rerun::Scalar(msg.imu_state.acc[1]));
    log->log("low_state_callback/imu/acc/z", rerun::Scalar(msg.imu_state.acc[2]));
    log->log("low_state_callback/imu/gyro/x", rerun::Scalar(msg.imu_state.gyro[0]));
    log->log("low_state_callback/imu/gyro/y", rerun::Scalar(msg.imu_state.gyro[1]));
    log->log("low_state_callback/imu/gyro/z", rerun::Scalar(msg.imu_state.gyro[2]));
}
/*这个方法的主要功能是处理从图像传感器接收到的图像消息，
并在配置指定的间隔内将图像记录到日志中。
这些图像可以用于后续的分析、调试或重放。
通过这种方式，Brain 节点能够记录视觉数据，
以便在需要时回放和检查机器人在特定时间点的视角。*/
void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{
    //检查配置中是否启用了重放日志记录
    if (!config->rerunLogEnable)
        return;
    //计数回调调用次数
    static int counter = 0;
    counter++;
    /*检查计数器是否达到了配置中设置的图像日志记录间隔（rerunLogImgInterval)
    如果是，则执行日志记录。*/
    if (counter % config->rerunLogImgInterval == 0)
    {
        //将图像数据转换为OpenCV格式：将ROS消息转换BGR格式OpenCV矩阵
        cv::Mat imageBGR(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        cv::Mat imageRGB;
        //将BGR图像转换为RGB格式：使用 cv::cvtColor 将BGR转换为RGB 
        cv::cvtColor(imageBGR, imageRGB, cv::COLOR_BGR2RGB);

        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10};
        //压缩图像：使用 cv::imencode 函数将RGB图像压缩为JPEG格式，并存储在 compressed_image 向量中。
        cv::imencode(".jpg", imageRGB, compressed_image, compression_params);
        //从图像消息的头信息中获取时间戳，并将其转换为秒
        double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        log->setTimeSeconds(time);
        //记录压缩后的图像到日志中，使用 rerun::EncodedImage 类型。
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}
//测试用
void Brain::headPoseCallback(const geometry_msgs::msg::Pose &msg)
{

    // --- for test:
    // if (config->rerunLogEnable) {
    if (false)
    {
        //提取头部姿态的位置和方向：从消息中提取头部位置的 x、y、z 坐标，以及方向（四元数）
        auto x = msg.position.x;
        auto y = msg.position.y;
        auto z = msg.position.z;

        auto orientation = msg.orientation;

        auto roll = rad2deg(atan2(2 * (orientation.w * orientation.x + orientation.y * orientation.z), 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)));
        auto pitch = rad2deg(asin(2 * (orientation.w * orientation.y - orientation.z * orientation.x)));
        auto yaw = rad2deg(atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y), 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)));

        log->setTimeNow();

        log->log("head_to_base/text",
                 rerun::TextLog("x: " + to_string(x) + " y: " + to_string(y) + " z: " + to_string(z) + " roll: " + to_string(roll) + " pitch: " + to_string(pitch) + " yaw: " + to_string(yaw)));
        log->log("head_to_base/x",
                 rerun::Scalar(x));
        log->log("head_to_base/y",
                 rerun::Scalar(y));
        log->log("head_to_base/z",
                 rerun::Scalar(z));
        log->log("head_to_base/roll",
                 rerun::Scalar(roll));
        log->log("head_to_base/pitch",
                 rerun::Scalar(pitch));
        log->log("head_to_base/yaw",
                 rerun::Scalar(yaw));
    }
}
/*将视觉检测消息中的数据转换为 Brain 节点可以理解和处理的格式，
包括坐标系的转换和对象信息的提取。
这些转换后的数据可以用于机器人的决策制定、导航和交互*/
vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;
    //从检测消息中获取时间戳
    auto timestamp = detections.header.stamp;
    //将时间戳转换为 rclcpp::Time 对象。
    rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        //为每个检测到的对象创建一个 GameObject 结构体 gObj
        GameObject gObj;
        //设置 gObj 的时间点、标签、边界框和置信度
        gObj.timePoint = timePoint;
        gObj.label = obj.label;

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;
        /*设置 gObj 的位置：如果对象的位置不为空且不是零向量，
        则使用对象的位置；否则，使用对象的投影位置。*/
        if (obj.position.size() > 0 && !(obj.position[0] == 0 && obj.position[1] == 0))
        {
            gObj.posToRobot.x = obj.position[0];
            gObj.posToRobot.y = obj.position[1];
        }
        else
        {
            gObj.posToRobot.x = obj.position_projection[0];
            gObj.posToRobot.y = obj.position_projection[1];
        }
        //计算对象到机器人的距离（range）、偏航角（yawToRobot）和俯仰角
        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range);
        //将对象的位置从机器人坐标系转换到场地坐标系
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);
        //将填充好的 gObj 添加到结果向量 res 中
        res.push_back(gObj);
    }
    //返回包含所有 GameObject 结构体的向量
    return res;
}
/*确定检测到的球中哪一个最有可能是真正的球.
计算球的位置，并更新 Brain 节点的状态以反映这一信息*/
void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    // Parameters
    //设置置信度阈值、俯仰角限制、时间计数阈值、检测计数阈值和置信度差异阈值。
    const double confidenceValve = 0.35;        // If the confidence is lower than this threshold, it is considered not a ball (note that the target confidence passed in by the detection module is currently all > 0.2).
    const double pitchLimit = deg2rad(0);       // When the pitch of the ball relative to the front of the robot (downward is positive) is lower than this value, it is considered not a ball. (Because the ball won't be in the sky.)
    const int timeCountThreshold = 5;           // Only when the ball is detected in consecutive several frames is it considered a ball. This is only used in the ball-finding strategy.
    const unsigned int detectCntThreshold = 3;  // The maximum count. Only when the target is detected in such a number of frames is it considered that the target is truly identified. (Currently only used for ball detection.)
    const unsigned int diffConfidThreshold = 4; // The threshold for the difference times between the tracked ball and the high-confidence ball. After reaching this threshold, the high-confidence ball will be adopted.
    //初始化变量
    double bestConfidence = 0;
    double minPixDistance = 1.e4;
    int indexRealBall = -1;  // Which ball is considered to be the real one. -1 indicates that no ball has been detected.
    int indexTraceBall = -1; // Track the ball according to the pixel distance. -1 indicates that no target has been tracked.

    // Find the most likely real ball.
    /*遍历球对象：遍历 ballObjs 向量，对每个球对象执行以下操作：
    如果球的置信度低于阈值 confidenceValve，则跳过该球。
    如果球的位置超出预设的范围（例如，太靠近或太远），则跳过该球。
    如果球的置信度高于当前最高置信度 bestConfidence，则更新 bestConfidence 和 indexRealBall。*/
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];

        // Judgment: If the confidence is too low, it is considered a false detection.
        if (ballObj.confidence < confidenceValve)
            continue;

        // Prevent the lights in the sky from being recognized as balls.
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 10.0)
            continue;

        // Find the one with the highest confidence among the remaining balls.
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }
    /*确定真实球：如果找到置信度最高的球，则将其标记为检测到的球，
    并更新 data->ballDetected 为 true，同时更新 data->ball 为该球对象。*/
    if (indexRealBall >= 0)
    {
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];
        //更新决策树：在决策树中设置 ball_location_known 条目为 true
        tree->setEntry<bool>("ball_location_known", true);
    }
    //将 data->ballDetected 设置为 false，并重置 data->ball 的相关成员
    else
    {
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;
        data->ball.confidence = 0;
    }
    //计算机器人到球的角度 robotBallAngleToField，这是根据球在场地坐标系中的位置与机器人的位置之差计算得出的
    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
}
/*筛选出置信度足够高的标记对象，
并更新 Brain 节点的状态以反映这些标记的位置。
这有助于机器人更好地理解其周围环境，
特别是在需要识别和响应特定标记（如球场线、角球点等）的情况下*/
void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    //设置一个参数 confidenceValve，这是一个置信度阈值，用于过滤标记对象
    const double confidenceValve = 0.1;
    //清空 data->markings 向量，该向量用于存储最终确认的标记对象
    data->markings.clear();
    /*遍历标记对象：遍历 markingObjs 向量，对每个标记对象执行以下操作：
    如果标记的置信度低于阈值 confidenceValve，则跳过该标记。
    如果标记的位置超出预设的范围（例如，太靠近或太远），则跳过该标记。*/
    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        if (marking.confidence < confidenceValve)
            continue;

        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)
            continue;
        //如果标记通过了上述过滤条件，则将其添加到 data->markings 向量中
        data->markings.push_back(marking);
    }
}