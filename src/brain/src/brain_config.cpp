#include "brain_config.h"
#include "utils/print.h"
/*用“throw表达式”来抛出异常,invalid_argument是异常类型
表示当使用了无效的参数时，会抛出该异常*/
void BrainConfig::handle()
{
    // playerStartPos[left, right]
    if (playerStartPos != "left" && playerStartPos != "right")
    {
        throw invalid_argument("palyer_start_pos must be one of [left, right]. Got: " + playerStartPos);
    }

    // playerRole [striker, goal_keeper]
    if (playerRole != "striker" && playerRole != "goal_keeper")
    {
        throw invalid_argument("player_role must be one of [striker, goal_keeper]. Got: " + playerRole);
    }

    // playerId [0, 1, 2, 3]
    if (playerId != 0 && playerId != 1 && playerId != 2 && playerId != 3)
    {
        throw invalid_argument("[Error] player_id must be one of [0, 1, 2, 3]. Got: " + to_string(playerId));
    }

    // fieldType [adult_size, kid_size]
    if (fieldType == "adult_size")
    {
        fieldDimensions = FD_ADULTSIZE;
    }
    else if (fieldType == "kid_size")
    {
        fieldDimensions = FD_KIDSIZE;
    }
    else
    {
        throw invalid_argument("[Error] fieldType must be one of [adult_size, kid_size]. Got: " + fieldType);
    }
}
/*将类的配置信息输出到一个ostream对象（通常是cout或文件流）。
这个函数接受一个ostream类型的引用作为参数，然后使用这个引用来输出配置信息*/
void BrainConfig::print(ostream &os)
{
    os << "Configs:" << endl;
    os << "----------------------------------------" << endl;
    os << "teamId = " << teamId << endl;
    os << "playerId = " << playerId << endl;
    os << "fieldType = " << fieldType << endl;
    os << "playerRole = " << playerRole << endl;
    os << "playerStartPos = " << playerStartPos << endl;
    os << "----------------------------------------" << endl;
    os << "robotHeight = " << robotHeight << endl;
    os << "robotOdomFactor = " << robotOdomFactor << endl;
    os << "vxFactor = " << vxFactor << endl;
    os << "yawOffset = " << yawOffset << endl;
    os << "----------------------------------------" << endl;
    os << "rerunLogEnable = " << rerunLogEnable << endl;
    os << "rerunLogServerAddr = " << rerunLogServerAddr << endl;
    os << "rerunLogImgInterval = " << rerunLogImgInterval << endl;
    os << "----------------------------------------" << endl;
    os << "treeFilePath = " << treeFilePath << endl;
    os << "----------------------------------------" << endl;
}