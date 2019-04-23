#ifndef CHOMP_UTILS_H_
#define CHOMP_UTILS_H_

#include <eigen3/Eigen/Core>
#include <vector>
//机械臂第一轴第二轴的长度

typedef double double_type;

static const double_type JOINT_LENGTH[2] = {0.325, 0.275};
static const double_type JOINT_LIMIT_MIN[4] = {-2.0595, -2.618, -0.15, -6.2832};
static const double_type JOINT_LIMIT_MAX[4] = {2.0595, 2.618, 0, 6.2832};
static const double_type END_HAND_LENGTH = 0.55;
static const double_type END_HAND_WIDTH = 0.21;

static const int NUM_JOINTS = 4;
static const int NUM_JOINTS_PLAN = 3;

static const int NUM_POSITION_PLAN = 3;

static const int DIFF_RULE_LENGTH = 7;
static const int NUM_DIFF_RULES = 3;

// 差分规则，中间点在最中间
static const double_type DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] = {
    {0, 0, -2 / 6.0, -3 / 6.0, 6 / 6.0, -1 / 6.0, 0},               // 速度
    {0, -1 / 12.0, 16 / 12.0, -30 / 12.0, 16 / 12.0, -1 / 12.0, 0}, // 加速度
    {0, 1 / 12.0, -17 / 12.0, 46 / 12.0, -46 / 12.0, 17 / 12.0, -1 / 12.0} // 加加速度
};


#endif