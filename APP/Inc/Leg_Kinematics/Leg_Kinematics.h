#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_LEG_KINEMATICS_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_LEG_KINEMATICS_H

typedef struct
{
  float x;
  float y;
} Leg_PointTypeDef;

typedef struct
{
  float theta1;
  float theta2;
} Leg_JointAnglesTypeDef;

#define LEG_LINK_L1_MM              130.0f
#define LEG_LINK_L2_MM              260.0f
#define LEG_REDUCTION_RATIO         6.33f
#define LEG_PI                      3.14159265358979323846f
#define LEG_TWO_PI                  6.28318530717958647692f
#define LEG_KIN_EPSILON             0.000001f

float clampf(float value, float min_value, float max_value);
float absf_local(float value);
int   Leg_Kinematics_Forward(const Leg_JointAnglesTypeDef *angles, Leg_PointTypeDef *foot);
int   Leg_Kinematics_Inverse(const Leg_PointTypeDef *foot, Leg_JointAnglesTypeDef *angles);

#endif
