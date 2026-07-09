#include "Leg_Kinematics.h"
#include <math.h>

float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

float absf_local(float value)
{
  return value < 0.0f ? -value : value;
}

float rotor_wrap_delta(float angle, float reference)
{
  float delta = angle - reference;
  while (delta > LEG_PI)
    delta -= LEG_TWO_PI;
  while (delta < -LEG_PI)
    delta += LEG_TWO_PI;
  return delta;
}

int Leg_Kinematics_Forward(const Leg_JointAnglesTypeDef *angles, Leg_PointTypeDef *foot)
{
  if (angles == NULL || foot == NULL)
    return 0;

  float theta1 = angles->theta1;
  float theta2 = angles->theta2;
  float cos1 = cosf(theta1);
  float cos2 = cosf(theta2);
  float sin1 = sinf(theta1);
  float sin2 = sinf(theta2);
  float cos_sum = cosf(theta1 + theta2);
  float d_sq = LEG_LINK_L1_MM * LEG_LINK_L1_MM * (2.0f + 2.0f * cos_sum);
  if (d_sq <= LEG_KIN_EPSILON)
    return 0;

  float h_sq = LEG_LINK_L2_MM * LEG_LINK_L2_MM - 0.25f * d_sq;
  if (h_sq < -LEG_KIN_EPSILON)
    return 0;
  if (h_sq < 0.0f)
    h_sq = 0.0f;

  float d = sqrtf(d_sq);
  float h = sqrtf(h_sq);
  float x_e = 0.5f * LEG_LINK_L1_MM * (cos2 - cos1);
  float y_e = 0.5f * LEG_LINK_L1_MM * (sin1 + sin2);
  float h_over_d = h / d;

  foot->x = x_e + h_over_d * LEG_LINK_L1_MM * (sin1 - sin2);
  foot->y = y_e + h_over_d * LEG_LINK_L1_MM * (cos1 + cos2);
  return 1;
}

int Leg_Kinematics_Inverse(const Leg_PointTypeDef *foot, Leg_JointAnglesTypeDef *angles)
{
  if (foot == NULL || angles == NULL)
    return 0;

  float x = foot->x;
  float y = foot->y;
  float r_sq = x * x + y * y;
  if (r_sq <= LEG_KIN_EPSILON)
    return 0;

  float r = sqrtf(r_sq);
  float max_reach = LEG_LINK_L1_MM + LEG_LINK_L2_MM;
  float min_reach = absf_local(LEG_LINK_L2_MM - LEG_LINK_L1_MM);
  if (r > max_reach || r < min_reach)
    return 0;

  float cos_alpha = (LEG_LINK_L1_MM * LEG_LINK_L1_MM + r_sq - LEG_LINK_L2_MM * LEG_LINK_L2_MM) /
                    (2.0f * LEG_LINK_L1_MM * r);
  if (cos_alpha > 1.0f)
    cos_alpha = 1.0f;
  else if (cos_alpha < -1.0f)
    cos_alpha = -1.0f;

  float phi = atan2f(y, x);
  float alpha = acosf(cos_alpha);
  angles->theta2 = phi - alpha;
  angles->theta1 = LEG_PI - alpha - phi;
  return 1;
}
