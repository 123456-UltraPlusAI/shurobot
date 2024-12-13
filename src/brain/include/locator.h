/**
 * @file locator.h
 * @brief It is carried out using the particle filter algorithm and locates (itself) through the landmark points on the pitch.
 */
/**
*@file locator.h
它是使用粒子滤波算法进行的，并通过球场上的地标点定位（自身）。
*/
#pragma once

#include <Eigen/Core>
#include <cstdlib> // for srand and rand
#include <ctime>   // for time
#include <limits>
#include <cmath>
#include <chrono>

#include "types.h"

// #define EPSILON 1e-5

using namespace std;
namespace chr = std::chrono;
// 定义了一些结构体和类来表示二维姿态、场地标记和定位结果。
struct PoseBox2D
{
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double thetamin;
	double thetamax;
};
//不同类型的地标点: L|T|X|P,P是点球
struct FieldMarker
{
	char type;			// L|T|X|P, representing different types of landmark points, where P represents the penalty mark.
	double x, y; // The position of the landmark point (in meters).
	double confidence;	// The recognition confidence level.
};

// 定位结果。
struct LocateResult
{
	bool success = false;
	// 0: 成功
	// 1: 未能生成新粒子（数量为0）
	// 2: 收敛后的残差误差不合理
	// 3: 未收敛
	// 4: 标记点数量不足
	// 5: 所有粒子的概率太低
	// -1代表初始状态
	int code = -1;
	double residual = 0; // 平均残差误差。
	Pose2D pose;
	int msecs = 0;  // 定位所消耗的时间。
};

/**
 * @class Locator
 * @brief 使用粒子滤波定位机器人。
 */
class Locator
{
public:
	// Parameters
	 // 当所有假设的x、y和theta范围小于此值时，认为已收敛。
	double convergeTolerance = 0.2; 
	// 如果每个标记的平均残差大于此值，则认为收敛位置不合理。
	double residualTolerance = 0.4; 
	// 最大迭代次数。
	double maxIteration = 20;	
	// 大致认为残差分布的mu是min(residuals) - muOffset * std(residuals)。
	double muOffset = 2.0;		
	// 每次重采样时，粒子数量为上一次的这一比例。
	double numShrinkRatio = 0.85;	
	// 每次重采样时，x、y和theta的偏移量减少到上一次的这一比例。
	double offsetShrinkRatio = 0.8;   
	// 所需的最小标记点数量
	double minMarkerCnt = 3;		

	// 数据存储
	vector<FieldMarker> fieldMarkers;
	FieldDimensions fieldDimensions;
	/* 一个n*5矩阵，用于存储计算结果，其中n是假设的数量。
	第0、1、2列是假设的姿态（x, y, theta），第3列是残差，
	第4列是归一化概率，第5列是归一化概率的累积和
	（从行0到当前行的总和）。*/
	Eigen::ArrayXXd hypos;				  // An n * 5 matrix used to store the calculation results, where n is the number of hypotheses. Columns 0, 1, and 2 are the hypothesized Pose (x, y, theta), column 3 is the residual, column 4 is the normalized probability, and column 5 is the cumulative sum of the normalized probabilities (the sum from row 0 to the current row).
	//定位的范围限制。
	PoseBox2D constraints;				  // The range constraint for positioning.
	// 生成新粒子时，与上一轮粒子的随机偏移范围。即新x在旧x的[-offsetX, offsetX]范围内随机。
	double offsetX, offsetY, offsetTheta; // When generating new particles, the random offset range from the particles in the previous round. That is, the new x is randomly within the range of [-offsetX, offsetX] of the old x.
	// 每次定位的最佳假设位置
	Pose2D bestPose;					  // The best hypothesized position for each positioning.
	// 每次定位的最小残差
	double bestResidual;				  // The minimum residual for each positioning.

	void init(FieldDimensions fd, int minMarkerCnt = 4, double residualTolerance = 0.4, double muOffsetParam = 2.0);

	/**
	 * @brief 根据场地尺寸信息生成场地上所有地标点的场地坐标系位置。
	 *
	 * @param fieldDimensions FieldDimensions, 场地尺寸信息。
	 *
	 */
	void calcFieldMarkers(FieldDimensions fd);

	/**
	 * @brief 计算机器人在场地上的姿态并返回整数代码。
	 *
	 * @param fieldDimensions FieldDimensions, 场地尺寸信息。
	 * @param markers_r vector<FieldMarker>, 通过视觉获得的机器人坐标系中场地地标点的位置。
	 * @param constraints PoseBox2D, 定位的先验约束条件。约束条件越严格，定位越快。
	 * @param pose &Pose2D, 定位的输出结果。
	 *
	 * @return int, 0: 成功；1: 未能生成新粒子（数量为0）；2: 收敛后的残差误差不合理；3: 未收敛；4: 标记点数量不足；5: 所有粒子的概率太低。
	 *
	 */
	int locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, Pose2D &pose, double &residual, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	/**
	 * @brief 计算机器人在场地上的姿态并返回结构体结果。
	 *
	 * @param fieldDimensions FieldDimensions, 场地尺寸信息。
	 * @param markers_r vector<FieldMarker>, 通过视觉获得的机器人坐标系中场地地标点的位置。
	 * @param constraints PoseBox2D, 定位的先验约束条件。约束条件越严格，定位越快。
	 *
	 * @return LocateResult
	 *         success: bool
	 *         code: int, 0: 成功；1: 未能生成新粒子（数量为0）；2: 收敛后的残差误差不合理；3: 未收敛；4: 标记点数量不足；5: 所有粒子的概率太低。
	 *         residual: double, 残差误差。
	 *         Pose2D: 定位结果。
	 */
	LocateResult locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	/**
	 * @brief 生成初始粒子。
	 *
	 * @param constraints PoseBox2D, 在约束范围内生成假设的姿态。
	 * @param num int, 指定生成多少个假设。
	 *
	 * @return int, 0 表示成功，非0表示失败。
	 *
	 */
	int genInitialParticles(int num = 200);

	/**
	 * @brief 根据概率重采样生成新粒子。
	 *
	 * @return int, 0 表示成功，非0表示失败。
	 */
	int genParticles();

	/**
	 * @brief 根据机器人的姿态将机器人观测到的场地地标点从机器人坐标系转换到场地坐标系。
	 *
	 * @param FieldMarker marker
	 * @param Pose2D pose
	 *
	 * @return FieldMarker, 包含所有地标点的向量。
	 */
	FieldMarker markerToFieldFrame(FieldMarker marker, Pose2D pose);

	/**
	 * @brief 获取观测到的标记点与场地地图上所有标记点之间的最小距离。
	 *
	 * @param marker FieldMarker
	 *
	 * @return double 最小距离。
	 */
	double minDist(FieldMarker marker);

	/**
	 * @brief 获取观测到的标记点与场地地图上最近标记点之间的位姿偏移。
	 *
	 * @param marker FieldMarker
	 *
	 * @return vector<double> {dx, dy} 从最近标记点的x和y偏移量，有符号。dx = x(地图标记点) - x(观测标记点)。
	 */
	vector<double> getOffset(FieldMarker marker);

	/**
	 * @brief 计算一组观测到的标记点（在机器人坐标系中）与场地标记点之间的拟合残差。
	 *
	 * @param markers_r vector<FieldMarker> markers_r 观测到的标记点。
	 * @param pose Pose2D 机器人的姿态，用于将markers_r转换到场地坐标系。
	 *
	 * @return double 残差。
	 */
	double residual(vector<FieldMarker> markers_r, Pose2D pose);

	/**
	 * @brief 检查当前假设是否已收敛。
	 *
	 * @return bool
	 */
	bool isConverged();

	/**
	 * @brief 根据当前位置假设计算相应的概率并存储在成员probs中。
	 *
	 * @return int, 0 表示成功，非0表示失败。
	 */
	int calcProbs(vector<FieldMarker> markers_r);

	/**
	 * @brief 收敛后根据标记点的位置重新校准x和y。
	 *
	 * @return Pose2D, 校准后的姿态。
	 */
	Pose2D finalAdjust(vector<FieldMarker> markers_r, Pose2D pose);

	/**
	 * @brief 高斯分布概率密度函数。
	 *
	 * @param r double 观测值。
	 * @param mu double 分布的均值。
	 * @param sigma double 分布的标准差。
	 *
	 * @return double 概率密度。
	 */
	inline double probDesity(double r, double mu, double sigma)
	{
		if (sigma < 1e-5)
			return 0.0;
		return 1 / sqrt(2 * M_PI * sigma * sigma) * exp(-(r - mu) * (r - mu) / (2 * sigma * sigma));
	};

	/**
	 * @brief 将粒子（假设）记录到rerun。
	 */
	void logParticles();
};
