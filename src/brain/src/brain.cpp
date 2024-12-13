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
    //declare_parameter�����ڵ����еĲ���
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
    /*��Щ�����������󣬿���ͨ��ROS2�Ĳ����������������ã�
    Ҳ�����������ڵ�ʱͨ�������н��и��ǡ�
    ��Щ����ͨ���������ýڵ����Ϊ�������������ID�����ID�������˵ĸ߶ȡ���̼����ӵȡ�
    ͨ���ڹ��캯����������Щ������Brain ��������䷽���з��ʺ�ʹ����Щ������*/
}


/*���������ʼ���� Brain ���������Ҫ�����
�������˸���ROS2����Ķ����ߣ�
�Ա�ڵ���Խ��պʹ������Բ�ͬ�������ͽӿڵ����ݡ�
��Щ�����߽����� Brain �ڵ���ROS2�����е������ڵ����ͨ�źͽ�����*/
void Brain::init()
{
    // Make sure to load the configuration first, and then the config can be used.
    config = std::make_shared<BrainConfig>();
    loadConfig();//���� loadConfig �������������ļ���������Դ�������á�

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();

    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);

    locator->init(config->fieldDimensions, 4, 0.5);

    tree->init();//���� BrainTree ʵ���� init ��������ʼ����������

    client->init();//���� RobotClient ʵ���� init ��������ʼ�������˿ͻ��ˡ�

    log->prepare();//���� BrainLog ʵ���� prepare ������׼����־��¼��

    data->lastSuccessfulLocalizeTime = get_clock()->now();//�������ɹ���λ��ʱ�����

    //����һ�������ߣ����� "/joy" ���⣬���ڽ�����Ϸ�ֱ������롣
    joySubscription = create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bind(&Brain::joystickCallback, this, _1));
    //����һ�������ߣ����� "/robocup/game_controller" ���⣬���ڽ��ղ��л��������ݡ�
    gameControlSubscription = create_subscription<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 1, bind(&Brain::gameControlCallback, this, _1));
    //����һ�������ߣ����� "/booster_vision/detection" ���⣬���ڽ����Ӿ�������ݡ�
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", 1, bind(&Brain::detectionsCallback, this, _1));
    //����һ�������ߣ����� "/odometer_state" ���⣬���ڽ�����̼�״̬���ݡ�
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state", 1, bind(&Brain::odometerCallback, this, _1));
    //����һ�������ߣ����� "/low_state" ���⣬���ڽ��ջ����˵ײ�״̬���ݡ�
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", 1, bind(&Brain::lowStateCallback, this, _1));
    //����һ�������ߣ����� "/camera/camera/color/image_raw" ���⣬���ڽ���ԭʼͼ�����ݡ�
    imageSubscription = create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 1, bind(&Brain::imageCallback, this, _1));
    //����һ�������ߣ����� "/head_pose" ���⣬���ڽ���ͷ����̬���ݡ�
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose", 1, bind(&Brain::headPoseCallback, this, _1));
}


/*��δ����� Brain ��� loadConfig ������ʵ�֣�
�������ROS2�Ĳ�����������ȡ���ò�����
���洢�� BrainConfig ���ʵ���С�*/
void Brain::loadConfig()
{
    //�Ӳ���������������Ϣ���洢��config�����еĶ�Ӧ����
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
    config->handle();//���� BrainConfig ʵ���� handle �����������ȡ���Ĳ�����

    // debug after handle the parameters
    ostringstream oss;//����һ���ַ����� oss�����ڹ���������Ϣ��
    config->print(oss);//���� BrainConfig ʵ���� print ��������������Ϣ��ӡ���ַ����� oss �С�
    prtDebug(oss.str());//���� prtDebug ���������ַ������е�������Ϣ��Ϊ������Ϣ�����
}

/**
 * will be called in the Ros2 loop
 */
/*tick ����ͨ�������Ϊ��ROS2��ִ������executor��ѭ���ж���ִ�У�
��ȷ���ڵ��ܹ���Ӧ�����仯��ִ������
����������£�tick ����ͨ�����¼���;����������� Brain �ڵ����Ϊ��
���ַ�������ڵ���ÿ��ʱ�䲽������״̬�����������µ���Ϣ�������ߡ�*/
void Brain::tick()
{
    /*���� Brain ��� updateMemory ������
    ����������ܸ�����½ڵ���ڲ�״̬����䣬
    ���磬���´��������ݡ�������״̬�����������Ϣ��*/
    updateMemory();
    /*���� BrainTree ��ʵ���� tick ������
    ����������ܸ���������߼���
    ���ݵ�ǰ�Ļ�����״̬�ͻ�����Ϣ��������һ���ж���*/
    tree->tick();
}

/*�����������Ҫ�����Ǹ���������ص��ڴ���Ϣ��
�����ݵ�ǰ����Ϸ״̬�����λ���������Ƿ���Ҫ�ȴ����ֿ���*/
void Brain::updateMemory()
{
    updateBallMemory();//����������ص��ڴ���Ϣ���������λ�á�

    static Point ballPos;//������
    static rclcpp::Time kickOffTime;//�洢����ʼ��ʱ��
    /*��鵱ǰ��ɫ�Ƿ�Ϊ��striker����ǰ�棩��
    ������Ϸ״̬Ϊ��SET�������ã��Ҳ������ǵĿ��򷽣�
    �����油״̬Ϊ��SET���Ҳ������ǵ��油���򷽡�*/
    if (
        tree->getEntry<string>("player_role") == "striker" && ((tree->getEntry<string>("gc_game_state") == "SET" && !tree->getEntry<bool>("gc_is_kickoff_side")) || (tree->getEntry<string>("gc_game_sub_state") == "SET" && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side"))))
    {
        ballPos = data->ball.posToRobot;//���� ballPos Ϊ������ڻ����˵�λ��
        kickOffTime = get_clock()->now();//��¼��ǰʱ��Ϊ����ʱ��
        //�ھ�����������һ����Ŀ����ʾ�������ڵȴ����ֿ���
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    }
    //���֮ǰ�Ѿ������˵ȴ����ֿ����״̬��
    else if (tree->getEntry<bool>("wait_for_opponent_kickoff"))
    {
        //������λ���Ƿ����������仯������0.3�ף�������ʱ���Ѿ���ȥ��10��
        if (
            norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y) > 0.3 || (get_clock()->now() - kickOffTime).seconds() > 10.0)
        {
            //���ٵȴ����ֿ��򣬸��¾������е���Ӧ��Ŀ��
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }
}
/*��Ҫ�����Ǹ������λ����Ϣ��
���������λ�úͻ����˵ĵ�ǰ״̬�����¾���������־*/
void Brain::updateBallMemory()
{
    // update Pose to field from Pose to robot (based on odom)
    //�洢�ӻ���������ϵ����������ϵ��ת������
    double xfr, yfr, thetafr; // fr = field to robot
    /*���� yfr, xfr, thetafr��
    ʹ�����Ǻ����ͻ���������ڳ��ص���̬��data->robotPoseToField����
    �������λ�ôӻ���������ϵ����������ϵ��ת��������*/
    yfr = sin(data->robotPoseToField.theta) * data->robotPoseToField.x - cos(data->robotPoseToField.theta) * data->robotPoseToField.y;
    xfr = -cos(data->robotPoseToField.theta) * data->robotPoseToField.x - sin(data->robotPoseToField.theta) * data->robotPoseToField.y;
    thetafr = -data->robotPoseToField.theta;
    //�����λ�ôӳ�������ϵת��������������ϵ
    transCoord(
        data->ball.posToField.x, data->ball.posToField.y, 0,
        xfr, yfr, thetafr,
        data->ball.posToRobot.x, data->ball.posToRobot.y, data->ball.posToRobot.z);
    //�����򵽻����˵ľ��룬������洢�� data->ball.range �С�
    data->ball.range = sqrt(data->ball.posToRobot.x * data->ball.posToRobot.x + data->ball.posToRobot.y * data->ball.posToRobot.y);
    //�ھ�������������ľ�����Ŀ
    tree->setEntry<double>("ball_range", data->ball.range);
    //����������ڻ����˵�ƫ���Ǻ͸�����
    data->ball.yawToRobot = atan2(data->ball.posToRobot.y, data->ball.posToRobot.x);
    data->ball.pitchToRobot = asin(config->robotHeight / data->ball.range);

    // mark ball as lost if long time no see
    /*������Ƿ�ʧ��������ϴο�����������ʱ�䳬�������õ� memoryLength��
    �����λ����֪��Ŀ����Ϊ false���������Ϊ��ʧ��*/
    if (get_clock()->now().seconds() - data->ball.timePoint.seconds() > config->memoryLength)
    {
        tree->setEntry<bool>("ball_location_known", false);
        data->ballDetected = false;
    }

    // log mem ball pos
    //������־�ĵ�ǰʱ��
    log->setTimeNow();
    /*��¼���λ�õ���־�У�
    ʹ�� rerun::LineStrips2D ����ʾ���λ�ã�
    ���������λ���Ƿ���֪��������ɫ*/
    log->log("field/memball",
             rerun::LineStrips2D({
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x - 0.2, -data->ball.posToField.y}, {data->ball.posToField.x + 0.2, -data->ball.posToField.y}},
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x, -data->ball.posToField.y - 0.2}, {data->ball.posToField.x, -data->ball.posToField.y + 0.2}},
                                 })
                 .with_colors({tree->getEntry<bool>("ball_location_known") ? 0xFFFFFFFF : 0xFF0000FF})
                 .with_radii({0.005})
                 .with_draw_order(30));
}
/*ȷ���򵽶Է�����������ԽǶȣ����������׼�����Լ���ε��������Ų���
margin ���������������ǻ���������ʱ����Χ������ƶ��ռ䡣*/
vector<double> Brain::getGoalPostAngles(const double margin)
{
    //�洢�Է�������������
    double leftX, leftY, rightX, rightY;
    //���ݳ��صĳߴ����öԷ���������Ĭ������
    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;

    /*�������������꣺����ҵ��Է������������"oppo-left"����
    ����� leftX �� leftY������ҵ��Է��Ҳ���������"oppo-right"����
    ����� rightX �� rightY*/
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
    /*����Ƕ� theta_l �� theta_r��
    ʹ�� atan2 ������������λ�õ������Ҳ��������ĽǶȣ�
    ͬʱ���Ǵ���� margin ������*/
    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);
    //����һ�����������Ƕȵ� vector<double> �����ء�
    vector<double> vec = {theta_l, theta_r};
    return vec;
}
/*У׼�͸��»����ˡ���Ͷ��ֵ�λ����Ϣ��
ȷ�����е�λ�����ݶ��ǻ������µĳ�������ϵ��
����ڻ����˵ĵ�������λ�;���������Ҫ��
��Ϊ��������׼ȷ�ĳ���������ִ������*/
//x,y,theat����������ڳ�������ϵ�µ�x���ꡢy�������̬��
void Brain::calibrateOdom(double x, double y, double theta)
{
    //�洢����̼�����ϵ������������ϵ��ת������
    double x_or, y_or, theta_or; // or = odom to robot
    //������̼Ƶ�λ�ôӻ���������ϵ����̼�����ϵ��ת������
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;
    //����̼Ƶ�λ�ôӻ���������ϵת������������ϵ��
    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);
    //�������˵�λ�ô���̼�����ϵת������������ϵ��
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
    //��ʱ�洢�����ĵ�z�������ֵ̬
    double placeHolder;
    // ball
    //�����λ�ôӻ���������ϵת������������ϵ
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder);

    // opponents
    //�����ֵ�λ�ôӻ���������ϵת������������ϵ
    for (int i = 0; i < data->opponents.size(); i++)
    {
        auto obj = data->opponents[i];
        transCoord(
            obj.posToRobot.x, obj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            obj.posToField.x, obj.posToField.y, placeHolder);
    }
}
/*��ȡ��ĳ���ض�ʱ��㵽���ھ�����ʱ�䣨�Ժ���Ϊ��λ��
������ϴμ�⵽ĳ���¼������Ƿ񳬹����ض���ʱ����ֵ*/
double Brain::msecsSince(rclcpp::Time time)
{
    return (this->get_clock()->now() - time).nanoseconds() / 1e6;
}
//����ң���������֣�
void Brain::joystickCallback(const sensor_msgs::msg::Joy &msg)
{
    //����Ƿ�û�а����κμ����LT, RT, LB, RB��
    if (msg.buttons[BTN_LT] == 0 && msg.buttons[BTN_RT] == 0 && msg.buttons[BTN_LB] == 0 && msg.buttons[BTN_RB] == 0)
    {
        //��鰴ť B �Ƿ񱻰���
        if (msg.buttons[BTN_B] == 1)
        {
            //�����ť B �����£��ھ����������� B_pressed ��ĿΪ true��
            tree->setEntry<bool>("B_pressed", true);
            prtDebug("B is pressed");
        }
        else if (msg.buttons[BTN_B] == 0)
        {
            //�����ť B ���ͷţ��ھ����������� B_pressed ��ĿΪ false��
            tree->setEntry<bool>("B_pressed", false);
            prtDebug("B is released");
        }
    }
    //���������LT���Ƿ񱻰���
    else if (msg.buttons[BTN_LT] == 1)
    {
        //����Ҳ�ҡ�˵� X ��� Y ���Ƿ�������
        if (msg.axes[AX_DX] || msg.axes[AX_DY])
        {
            //��ҡ�˵� X ����������ٶ����� vxFactor��
            config->vxFactor += 0.01 * msg.axes[AX_DX];
            //������ҡ�˵� Y ���������ƫ��ƫ�� yawOffset��
            config->yawOffset += 0.01 * msg.axes[AX_DY];
            prtDebug(format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset));
        }
        //��鰴ť X �Ƿ񱻰���
        if (msg.buttons[BTN_X] == 1)
        {
            //�ھ����������ÿ���״̬Ϊ 1
            tree->setEntry<int>("control_state", 1);
            //��������ʹ������ֹͣ�ƶ�
            client->setVelocity(0., 0., 0.);
            ��������ʹ������ͷ����λ
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
        }
        else if (msg.buttons[BTN_A] == 1)
        {
            //�ھ����������ÿ���״̬Ϊ 2
            tree->setEntry<int>("control_state", 2);
            //�ھ�������������̼�У׼״̬ΪδУ׼��
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
        }
        else if (msg.buttons[BTN_B] == 1)
        {
            //�ھ����������ÿ���״̬Ϊ 3
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
        }
        else if (msg.buttons[BTN_Y] == 1)
        {
            //�л���ɫ�������ǰ��ɫ�ǡ�striker����
            // ���л�Ϊ��goal_keeper������֮��Ȼ
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
        }
    }
}
/*����Ӳ��л����յ�����Ϣ��
��������Ϣ���¾������е���Ϸ״̬����״̬��������Ϣ�͵÷���Ϣ*/
void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    //��ȡ������Ϸ״̬���Ӿ������л�ȡ���� gc_game_state
    auto lastGameState = tree->getEntry<string>("gc_game_state");
    //������Ϸ״̬ӳ�䣺����һ���������п�����Ϸ״̬������ gameStateMap
    vector<string> gameStateMap = {
        "INITIAL", // Initialization state, players are ready outside the field.
        "READY",   // Ready state, players enter the field and walk to their starting positions.
        "SET",     // Stop action, waiting for the referee machine to issue the instruction to start the game.
        "PLAY",    // Normal game.
        "END"      // The game is over.
    };
    //��ȡ��ǰ����Ϸ״̬��������洢�� gameState �����С�
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    //���¾������е���Ϸ״̬���ھ����������õ�ǰ����Ϸ״̬
    tree->setEntry<string>("gc_game_state", gameState);
    /*������Ϣ�е� kick_off_team �ֶκ������е� teamId��
    �жϵ�ǰ�����Ƿ�Ϊ���򷽣����ھ�������������Ӧ����Ŀ��*/
    bool isKickOffSide = (msg.kick_off_team == config->teamId);
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);
    /*��ȡ��Ϸ��״̬���ͣ�������Ϣ�е� secondary_state �ֶΣ�
    �ж���Ϸ��״̬���ͣ����ھ�������������Ӧ����Ŀ��
    ������Ϸ��״̬ӳ�䣺����һ���������п�����Ϸ��״̬������ gameSubStateMap��*/
    string gameSubStateType = static_cast<int>(msg.secondary_state) == 0 ? "NONE" : "FREE_KICK";
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};
    //��ȡ��ǰ��Ϸ��״̬
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    //�Ƿ�Ϊ��״̬�Ŀ���
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == config->teamId);
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);
    /*��ȡ������Ϣ�����������е� teamId��
    ����Ϣ�е� teams �ֶ��л�ȡ���ӵ���Ϣ��
    ������洢�� myTeamInfo ������*/
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
    /*���µ�����Ϣ�����ݱ�����Ϣ������ data->penalty ���飬
    ���ھ������������Ƿ��ڵ���״̬*/
    data->penalty[0] = static_cast<int>(myTeamInfo.players[0].penalty);
    data->penalty[1] = static_cast<int>(myTeamInfo.players[1].penalty);
    data->penalty[2] = static_cast<int>(myTeamInfo.players[2].penalty);
    data->penalty[3] = static_cast<int>(myTeamInfo.players[3].penalty);
    double isUnderPenalty = (data->penalty[config->playerId] != 0);
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);
    //���µ÷���Ϣ�����ݱ�����Ϣ�����µ÷֣����ھ������������Ƿ�ոյ÷�
    int curScore = static_cast<int>(myTeamInfo.score);
    if (curScore > data->lastScore)
    {
        tree->setEntry<bool>("we_just_scored", true);
        data->lastScore = curScore;
    }
    //���øոյ÷ֵ�״̬�������Ϸ״̬Ϊ "SET"�����ھ����������øոյ÷ֵ�״̬��
    if (gameState == "SET")
    {
        tree->setEntry<bool>("we_just_scored", false);
    }
}
/*������Ӿ�ϵͳ���յ��ļ����Ϣ������⵽����Ϸ������࣬
����¼��ص���־��Ϣ*/
void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    //����Ϣ����ȡ��Ϸ����
    auto gameObjects = getGameObjects(msg);

    //���������Ϸ������������������ڴ洢�����������ˡ������ˡ��ϰ���ͱ�ǵ�����
    vector<GameObject> balls, goalPosts, persons, robots, obstacles, markings;
    //������Ϸ���󣺱��� gameObjects ���������ݶ���ı�ǩ�����Ƿ��ൽ��Ӧ��������
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
    //�����⵽����
    detectProcessBalls(balls);
    //�����⵽�ı�ǡ�
    detectProcessMarkings(markings);
    //�����־�Ƿ����ã������־δ���ã��򷵻�
    if (!log->isEnabled())
        return;

    // log detection boxes to rerun
    //��ȡ��ǰʱ�䣺��ȡ��ǰʱ�䣬������־��¼
    auto detection_time_stamp = msg.header.stamp;
    rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto now = get_clock()->now();
    //��������ɫӳ�䣺����һ��ӳ�䣬����ͬ�ı�ǩӳ�䵽��ͬ����ɫ
    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
    };

    // for logging boundingBoxes
    //����������־��¼���������������ڴ洢��С�㡢��С����ǩ����ɫ������
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame
    //���� gameObjects ������Ϊÿ������������־��¼���������
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
    //��¼�����ϵļ��㣺ʹ�� rerun::Points2D ���¼�����ϵļ��㡣
    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
             // .with_labels(labels)
    );
    //��¼�����ϵļ��㣺ʹ�� rerun::Points2D ���¼�����ϵļ��㡣
    log->log("robotframe/detection_points",
             rerun::Points2D(points_r)
                 .with_colors(colors)
             // .with_labels(labels)
    );
}
/*�������̼ƽ��յ���λ����Ϣ������ת��Ϊ��������ϵ�µ�λ�ã�
����¼����־�У��Ա�����ķ����Ϳ��ӻ���
ͨ�����ַ�ʽ��Brain �ڵ��ܹ�����ʵʱ����̼����ݸ�����Ի�����λ�õ���⡣*/
void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{
    /*���»������������̼Ƶ�λ�ã�
    �����յ�����̼���Ϣ�е� x��y �� theta ֵ���������е� robotOdomFactor*/
    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;
    //�������˵�λ�ô���̼�����ϵת������������ϵ
    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);
    /*��¼�������ڳ����ϵ�λ�á�ʹ�� rerun::Points2D ������¼�����㣺һ���ǻ����˵�����λ�ã���һ���ǻ����˵�һ������ָʾ�㣨ͨ���ǻ����˵�ǰ�ˣ���
����ָʾ��ļ��㣺ʹ�û����˵���̬ theta ���㷽��ָʾ���λ�ã������λ�ڻ���������λ��ǰ��0.1�״����������Ż����˵�ǰ������
���õ�İ뾶��Ϊ���������ð뾶�����ĵ�뾶Ϊ0.2�ף�����ָʾ��뾶Ϊ0.1�ס�
���õ����ɫ��Ϊ������������ɫ�����ĵ���ɫΪ��ɫ��0xFF6666FF��������ָʾ����ɫΪ��ɫ��0xFF0000FF����*/
    log->setTimeNow();
    log->log("field/robot",
             rerun::Points2D({{data->robotPoseToField.x, -data->robotPoseToField.y}, {data->robotPoseToField.x + 0.1 * cos(data->robotPoseToField.theta), -data->robotPoseToField.y - 0.1 * sin(data->robotPoseToField.theta)}})
                 .with_radii({0.2, 0.1})
                 .with_colors({0xFF6666FF, 0xFF0000FF}));
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    /*���»�����ͷ������̬��
    ����Ϣ����ȡͷ��ƫ����headYaw���͸�����headPitch���ĽǶ�*/
    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;

    log->setTimeNow();
    /*��¼IMU�����Բ�����Ԫ����״̬��ʹ�� log->log ������¼IMU��״̬��
    ��̬�ǣ�Roll, Pitch, Yaw������¼IMU��������̬�ǡ�
    ���ٶȣ�Acc������¼IMU���������ٶȷ�����
    �����ǣ�Gyro������¼IMU���������ٶȷ�����*/
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
/*�����������Ҫ�����Ǵ����ͼ�񴫸������յ���ͼ����Ϣ��
��������ָ���ļ���ڽ�ͼ���¼����־�С�
��Щͼ��������ں����ķ��������Ի��طš�
ͨ�����ַ�ʽ��Brain �ڵ��ܹ���¼�Ӿ����ݣ�
�Ա�����Ҫʱ�طźͼ����������ض�ʱ�����ӽǡ�*/
void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{
    //����������Ƿ��������ط���־��¼
    if (!config->rerunLogEnable)
        return;
    //�����ص����ô���
    static int counter = 0;
    counter++;
    /*���������Ƿ�ﵽ�����������õ�ͼ����־��¼�����rerunLogImgInterval)
    ����ǣ���ִ����־��¼��*/
    if (counter % config->rerunLogImgInterval == 0)
    {
        //��ͼ������ת��ΪOpenCV��ʽ����ROS��Ϣת��BGR��ʽOpenCV����
        cv::Mat imageBGR(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        cv::Mat imageRGB;
        //��BGRͼ��ת��ΪRGB��ʽ��ʹ�� cv::cvtColor ��BGRת��ΪRGB 
        cv::cvtColor(imageBGR, imageRGB, cv::COLOR_BGR2RGB);

        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10};
        //ѹ��ͼ��ʹ�� cv::imencode ������RGBͼ��ѹ��ΪJPEG��ʽ�����洢�� compressed_image �����С�
        cv::imencode(".jpg", imageRGB, compressed_image, compression_params);
        //��ͼ����Ϣ��ͷ��Ϣ�л�ȡʱ�����������ת��Ϊ��
        double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        log->setTimeSeconds(time);
        //��¼ѹ�����ͼ����־�У�ʹ�� rerun::EncodedImage ���͡�
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}
//������
void Brain::headPoseCallback(const geometry_msgs::msg::Pose &msg)
{

    // --- for test:
    // if (config->rerunLogEnable) {
    if (false)
    {
        //��ȡͷ����̬��λ�úͷ��򣺴���Ϣ����ȡͷ��λ�õ� x��y��z ���꣬�Լ�������Ԫ����
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
/*���Ӿ������Ϣ�е�����ת��Ϊ Brain �ڵ�������ʹ���ĸ�ʽ��
��������ϵ��ת���Ͷ�����Ϣ����ȡ��
��Щת��������ݿ������ڻ����˵ľ����ƶ��������ͽ���*/
vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;
    //�Ӽ����Ϣ�л�ȡʱ���
    auto timestamp = detections.header.stamp;
    //��ʱ���ת��Ϊ rclcpp::Time ����
    rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        //Ϊÿ����⵽�Ķ��󴴽�һ�� GameObject �ṹ�� gObj
        GameObject gObj;
        //���� gObj ��ʱ��㡢��ǩ���߽������Ŷ�
        gObj.timePoint = timePoint;
        gObj.label = obj.label;

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;
        /*���� gObj ��λ�ã���������λ�ò�Ϊ���Ҳ�����������
        ��ʹ�ö����λ�ã�����ʹ�ö����ͶӰλ�á�*/
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
        //������󵽻����˵ľ��루range����ƫ���ǣ�yawToRobot���͸�����
        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range);
        //�������λ�ôӻ���������ϵת������������ϵ
        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);
        //�����õ� gObj ��ӵ�������� res ��
        res.push_back(gObj);
    }
    //���ذ������� GameObject �ṹ�������
    return res;
}
/*ȷ����⵽��������һ�����п�������������.
�������λ�ã������� Brain �ڵ��״̬�Է�ӳ��һ��Ϣ*/
void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    // Parameters
    //�������Ŷ���ֵ�����������ơ�ʱ�������ֵ����������ֵ�����ŶȲ�����ֵ��
    const double confidenceValve = 0.35;        // If the confidence is lower than this threshold, it is considered not a ball (note that the target confidence passed in by the detection module is currently all > 0.2).
    const double pitchLimit = deg2rad(0);       // When the pitch of the ball relative to the front of the robot (downward is positive) is lower than this value, it is considered not a ball. (Because the ball won't be in the sky.)
    const int timeCountThreshold = 5;           // Only when the ball is detected in consecutive several frames is it considered a ball. This is only used in the ball-finding strategy.
    const unsigned int detectCntThreshold = 3;  // The maximum count. Only when the target is detected in such a number of frames is it considered that the target is truly identified. (Currently only used for ball detection.)
    const unsigned int diffConfidThreshold = 4; // The threshold for the difference times between the tracked ball and the high-confidence ball. After reaching this threshold, the high-confidence ball will be adopted.
    //��ʼ������
    double bestConfidence = 0;
    double minPixDistance = 1.e4;
    int indexRealBall = -1;  // Which ball is considered to be the real one. -1 indicates that no ball has been detected.
    int indexTraceBall = -1; // Track the ball according to the pixel distance. -1 indicates that no target has been tracked.

    // Find the most likely real ball.
    /*��������󣺱��� ballObjs ��������ÿ�������ִ�����²�����
    ���������Ŷȵ�����ֵ confidenceValve������������
    ������λ�ó���Ԥ��ķ�Χ�����磬̫������̫Զ��������������
    ���������Ŷȸ��ڵ�ǰ������Ŷ� bestConfidence������� bestConfidence �� indexRealBall��*/
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
    /*ȷ����ʵ������ҵ����Ŷ���ߵ���������Ϊ��⵽����
    ������ data->ballDetected Ϊ true��ͬʱ���� data->ball Ϊ�������*/
    if (indexRealBall >= 0)
    {
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];
        //���¾��������ھ����������� ball_location_known ��ĿΪ true
        tree->setEntry<bool>("ball_location_known", true);
    }
    //�� data->ballDetected ����Ϊ false�������� data->ball ����س�Ա
    else
    {
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;
        data->ball.confidence = 0;
    }
    //��������˵���ĽǶ� robotBallAngleToField�����Ǹ������ڳ�������ϵ�е�λ��������˵�λ��֮�����ó���
    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
}
/*ɸѡ�����Ŷ��㹻�ߵı�Ƕ���
������ Brain �ڵ��״̬�Է�ӳ��Щ��ǵ�λ�á�
�������ڻ����˸��õ��������Χ������
�ر�������Ҫʶ�����Ӧ�ض���ǣ������ߡ������ȣ��������*/
void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    //����һ������ confidenceValve������һ�����Ŷ���ֵ�����ڹ��˱�Ƕ���
    const double confidenceValve = 0.1;
    //��� data->markings ���������������ڴ洢����ȷ�ϵı�Ƕ���
    data->markings.clear();
    /*������Ƕ��󣺱��� markingObjs ��������ÿ����Ƕ���ִ�����²�����
    �����ǵ����Ŷȵ�����ֵ confidenceValve���������ñ�ǡ�
    �����ǵ�λ�ó���Ԥ��ķ�Χ�����磬̫������̫Զ�����������ñ�ǡ�*/
    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        if (marking.confidence < confidenceValve)
            continue;

        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)
            continue;
        //������ͨ������������������������ӵ� data->markings ������
        data->markings.push_back(marking);
    }
}