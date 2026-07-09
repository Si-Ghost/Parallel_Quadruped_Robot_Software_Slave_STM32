#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_LEG_GAIT_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_LEG_GAIT_H

#include <stdint.h>

int Leg_Gait_StartDebugTrace(uint8_t leg);
int Leg_Gait_StartAllMicroTest(void);
int Leg_Gait_StartPrepPoseTest(void);
int Leg_Gait_StartSineTest(uint8_t leg, float amplitude_mm, float freq_hz);
int Leg_Gait_StartTrotTest(void);

void Leg_Gait_ServiceDebugTrace(void);
void Leg_Gait_ServiceAllMicro(void);
void Leg_Gait_ServicePrepPose(void);
void Leg_Gait_ServiceSine(void);
void Leg_Gait_ServiceTrot(void);

int  Leg_Gait_AnyActive(void);
void Leg_Gait_StopAll(void);

#endif
