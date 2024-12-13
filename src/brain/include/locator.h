/**
 * @file locator.h
 * @brief It is carried out using the particle filter algorithm and locates (itself) through the landmark points on the pitch.
 */
/**
*@file locator.h
����ʹ�������˲��㷨���еģ���ͨ�����ϵĵر�㶨λ��������
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
// ������һЩ�ṹ���������ʾ��ά��̬�����ر�ǺͶ�λ�����
struct PoseBox2D
{
	double xmin;
	double xmax;
	double ymin;
	double ymax;
	double thetamin;
	double thetamax;
};
//��ͬ���͵ĵر��: L|T|X|P,P�ǵ���
struct FieldMarker
{
	char type;			// L|T|X|P, representing different types of landmark points, where P represents the penalty mark.
	double x, y; // The position of the landmark point (in meters).
	double confidence;	// The recognition confidence level.
};

// ��λ�����
struct LocateResult
{
	bool success = false;
	// 0: �ɹ�
	// 1: δ�����������ӣ�����Ϊ0��
	// 2: ������Ĳв�������
	// 3: δ����
	// 4: ��ǵ���������
	// 5: �������ӵĸ���̫��
	// -1�����ʼ״̬
	int code = -1;
	double residual = 0; // ƽ���в���
	Pose2D pose;
	int msecs = 0;  // ��λ�����ĵ�ʱ�䡣
};

/**
 * @class Locator
 * @brief ʹ�������˲���λ�����ˡ�
 */
class Locator
{
public:
	// Parameters
	 // �����м����x��y��theta��ΧС�ڴ�ֵʱ����Ϊ��������
	double convergeTolerance = 0.2; 
	// ���ÿ����ǵ�ƽ���в���ڴ�ֵ������Ϊ����λ�ò�����
	double residualTolerance = 0.4; 
	// ������������
	double maxIteration = 20;	
	// ������Ϊ�в�ֲ���mu��min(residuals) - muOffset * std(residuals)��
	double muOffset = 2.0;		
	// ÿ���ز���ʱ����������Ϊ��һ�ε���һ������
	double numShrinkRatio = 0.85;	
	// ÿ���ز���ʱ��x��y��theta��ƫ�������ٵ���һ�ε���һ������
	double offsetShrinkRatio = 0.8;   
	// �������С��ǵ�����
	double minMarkerCnt = 3;		

	// ���ݴ洢
	vector<FieldMarker> fieldMarkers;
	FieldDimensions fieldDimensions;
	/* һ��n*5�������ڴ洢������������n�Ǽ����������
	��0��1��2���Ǽ������̬��x, y, theta������3���ǲв
	��4���ǹ�һ�����ʣ���5���ǹ�һ�����ʵ��ۻ���
	������0����ǰ�е��ܺͣ���*/
	Eigen::ArrayXXd hypos;				  // An n * 5 matrix used to store the calculation results, where n is the number of hypotheses. Columns 0, 1, and 2 are the hypothesized Pose (x, y, theta), column 3 is the residual, column 4 is the normalized probability, and column 5 is the cumulative sum of the normalized probabilities (the sum from row 0 to the current row).
	//��λ�ķ�Χ���ơ�
	PoseBox2D constraints;				  // The range constraint for positioning.
	// ����������ʱ������һ�����ӵ����ƫ�Ʒ�Χ������x�ھ�x��[-offsetX, offsetX]��Χ�������
	double offsetX, offsetY, offsetTheta; // When generating new particles, the random offset range from the particles in the previous round. That is, the new x is randomly within the range of [-offsetX, offsetX] of the old x.
	// ÿ�ζ�λ����Ѽ���λ��
	Pose2D bestPose;					  // The best hypothesized position for each positioning.
	// ÿ�ζ�λ����С�в�
	double bestResidual;				  // The minimum residual for each positioning.

	void init(FieldDimensions fd, int minMarkerCnt = 4, double residualTolerance = 0.4, double muOffsetParam = 2.0);

	/**
	 * @brief ���ݳ��سߴ���Ϣ���ɳ��������еر��ĳ�������ϵλ�á�
	 *
	 * @param fieldDimensions FieldDimensions, ���سߴ���Ϣ��
	 *
	 */
	void calcFieldMarkers(FieldDimensions fd);

	/**
	 * @brief ����������ڳ����ϵ���̬�������������롣
	 *
	 * @param fieldDimensions FieldDimensions, ���سߴ���Ϣ��
	 * @param markers_r vector<FieldMarker>, ͨ���Ӿ���õĻ���������ϵ�г��صر���λ�á�
	 * @param constraints PoseBox2D, ��λ������Լ��������Լ������Խ�ϸ񣬶�λԽ�졣
	 * @param pose &Pose2D, ��λ����������
	 *
	 * @return int, 0: �ɹ���1: δ�����������ӣ�����Ϊ0����2: ������Ĳв�������3: δ������4: ��ǵ��������㣻5: �������ӵĸ���̫�͡�
	 *
	 */
	int locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, Pose2D &pose, double &residual, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	/**
	 * @brief ����������ڳ����ϵ���̬�����ؽṹ������
	 *
	 * @param fieldDimensions FieldDimensions, ���سߴ���Ϣ��
	 * @param markers_r vector<FieldMarker>, ͨ���Ӿ���õĻ���������ϵ�г��صر���λ�á�
	 * @param constraints PoseBox2D, ��λ������Լ��������Լ������Խ�ϸ񣬶�λԽ�졣
	 *
	 * @return LocateResult
	 *         success: bool
	 *         code: int, 0: �ɹ���1: δ�����������ӣ�����Ϊ0����2: ������Ĳв�������3: δ������4: ��ǵ��������㣻5: �������ӵĸ���̫�͡�
	 *         residual: double, �в���
	 *         Pose2D: ��λ�����
	 */
	LocateResult locateRobot(vector<FieldMarker> markers_r, PoseBox2D constraints, int numParticles = 200, double offsetX = 2.0, double offsetY = 2.0, double offsetTheta = M_PI / 4);

	/**
	 * @brief ���ɳ�ʼ���ӡ�
	 *
	 * @param constraints PoseBox2D, ��Լ����Χ�����ɼ������̬��
	 * @param num int, ָ�����ɶ��ٸ����衣
	 *
	 * @return int, 0 ��ʾ�ɹ�����0��ʾʧ�ܡ�
	 *
	 */
	int genInitialParticles(int num = 200);

	/**
	 * @brief ���ݸ����ز������������ӡ�
	 *
	 * @return int, 0 ��ʾ�ɹ�����0��ʾʧ�ܡ�
	 */
	int genParticles();

	/**
	 * @brief ���ݻ����˵���̬�������˹۲⵽�ĳ��صر��ӻ���������ϵת������������ϵ��
	 *
	 * @param FieldMarker marker
	 * @param Pose2D pose
	 *
	 * @return FieldMarker, �������еر���������
	 */
	FieldMarker markerToFieldFrame(FieldMarker marker, Pose2D pose);

	/**
	 * @brief ��ȡ�۲⵽�ı�ǵ��볡�ص�ͼ�����б�ǵ�֮�����С���롣
	 *
	 * @param marker FieldMarker
	 *
	 * @return double ��С���롣
	 */
	double minDist(FieldMarker marker);

	/**
	 * @brief ��ȡ�۲⵽�ı�ǵ��볡�ص�ͼ�������ǵ�֮���λ��ƫ�ơ�
	 *
	 * @param marker FieldMarker
	 *
	 * @return vector<double> {dx, dy} �������ǵ��x��yƫ�������з��š�dx = x(��ͼ��ǵ�) - x(�۲��ǵ�)��
	 */
	vector<double> getOffset(FieldMarker marker);

	/**
	 * @brief ����һ��۲⵽�ı�ǵ㣨�ڻ���������ϵ�У��볡�ر�ǵ�֮�����ϲв
	 *
	 * @param markers_r vector<FieldMarker> markers_r �۲⵽�ı�ǵ㡣
	 * @param pose Pose2D �����˵���̬�����ڽ�markers_rת������������ϵ��
	 *
	 * @return double �в
	 */
	double residual(vector<FieldMarker> markers_r, Pose2D pose);

	/**
	 * @brief ��鵱ǰ�����Ƿ���������
	 *
	 * @return bool
	 */
	bool isConverged();

	/**
	 * @brief ���ݵ�ǰλ�ü��������Ӧ�ĸ��ʲ��洢�ڳ�Աprobs�С�
	 *
	 * @return int, 0 ��ʾ�ɹ�����0��ʾʧ�ܡ�
	 */
	int calcProbs(vector<FieldMarker> markers_r);

	/**
	 * @brief ��������ݱ�ǵ��λ������У׼x��y��
	 *
	 * @return Pose2D, У׼�����̬��
	 */
	Pose2D finalAdjust(vector<FieldMarker> markers_r, Pose2D pose);

	/**
	 * @brief ��˹�ֲ������ܶȺ�����
	 *
	 * @param r double �۲�ֵ��
	 * @param mu double �ֲ��ľ�ֵ��
	 * @param sigma double �ֲ��ı�׼�
	 *
	 * @return double �����ܶȡ�
	 */
	inline double probDesity(double r, double mu, double sigma)
	{
		if (sigma < 1e-5)
			return 0.0;
		return 1 / sqrt(2 * M_PI * sigma * sigma) * exp(-(r - mu) * (r - mu) / (2 * sigma * sigma));
	};

	/**
	 * @brief �����ӣ����裩��¼��rerun��
	 */
	void logParticles();
};
