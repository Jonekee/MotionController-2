#include "kalman.h"
///************************************************************/
///*	
// *
// */
// 
//S_FLOAT_XYZ K_Angle;
//S_FLOAT_XYZ Angle_dot, angle_er;

//static float angle, angle_dot; 		
//const float Q_angle = 0.002, Q_gyro = 0.002, R_angle = 0.5, dt = 0.001;		//���ٶȼ�Э���� ������Э���� ϵͳЭ���� ����ʱ��	
//static float P[2][2]= {{ 1, 0 }, { 0, 1 }};	
//static float Pdot[4] = {0, 0, 0, 0};
//const u8 C_0 = 1;
//static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
///*************************************************

//���ƣ�void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)
//���ܣ���������������ٶȼ�����ͨ���˲��㷨�ں�
//���������
//  	float angle_m   ���ٶȼƼ���ĽǶ�
//		float gyro_m    �����ǽ��ٶ�
//		float *angle_f  �ںϺ�ĽǶ�
//		float *angle_dot_f  �ںϺ�Ľ��ٶ�
//����������˲���ĽǶȼ����ٶ�
//����ֵ����
//**************************************************/
//void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)			
//{
//  angle += (gyro_m - q_bias) * dt;	//�������  �����ǻ��ֵõ��ĽǶ�
//	
//  Pdot[0] = Q_angle - P[0][1] - P[1][0];		//Pk-����������Э�����΢��
//  Pdot[1] = -P[1][1];												
//  Pdot[2] = -P[1][1];
//  Pdot[3] = Q_gyro;
//	
//  P[0][0] += Pdot[0] * dt;		//Pk-����������Э����΢�ֵĻ���
//  P[0][1] += Pdot[1] * dt;		//=����������Э����
//  P[1][0] += Pdot[2] * dt;
//  P[1][1] += Pdot[3] * dt;
//	
//  angle_err = angle_m - angle;		//Zk-�������		�в�
//	
//  PCt_0=C_0 * P[0][0];
//  PCt_1=C_0 * P[1][0];
//	
//  E = R_angle + C_0 * PCt_0;
//	
//  K_0 = PCt_0 / E;
//  K_1 = PCt_1 / E;
//	
//  t_0 = PCt_0;
//  t_1 = C_0 * P[0][1];

//  P[0][0] -= K_0 * t_0;		//����������Э����
//  P[0][1] -= K_0 * t_1;
//  P[1][0] -= K_1 * t_0;
//  P[1][1] -= K_1 * t_1;
//		
//  angle	+= K_0 * angle_err;			//�������	
//  q_bias += K_1 * angle_err;		//�������
//  angle_dot = gyro_m - q_bias;	//���ֵ��������ƣ���΢��=���ٶ�

//  *angle_f = angle;							//����Ƕ�
//  *angle_dot_f = angle_dot;			//������ٶ�
//}


//void kalman_filter(S_FLOAT_XYZ angle_m, S_FLOAT_XYZ gyro_m, S_FLOAT_XYZ *angle_f, S_FLOAT_XYZ *angle_dot_f)			
//{
//  angle += (gyro_m.X - q_bias) * dt;	//�������  �����ǻ��ֵõ��ĽǶ�
//	angle += (gyro_m.Y - q_bias) * dt;
//	angle += (gyro_m.Z - q_bias) * dt;
//	
//  Pdot[0] = Q_angle - P[0][1] - P[1][0];		//Pk-����������Э�����΢��
//  Pdot[1] = -P[1][1];												
//  Pdot[2] = -P[1][1];
//  Pdot[3] = Q_gyro;
//	
//  P[0][0] += Pdot[0] * dt;		//Pk-����������Э����΢�ֵĻ���
//  P[0][1] += Pdot[1] * dt;		//=����������Э����
//  P[1][0] += Pdot[2] * dt;
//  P[1][1] += Pdot[3] * dt;
//	
//  angle_er.X = angle_m.X - K_Angle.X;		//Zk-�������		�в�
//	angle_er.Y = angle_m.Y - K_Angle.Y;
//	angle_er.Z = angle_m.Z - K_Angle.Z;
//	
//  PCt_0=C_0 * P[0][0];
//  PCt_1=C_0 * P[1][0];
//	
//  E = R_angle + C_0 * PCt_0;
//	
//  K_0 = PCt_0 / E;
//  K_1 = PCt_1 / E;
//	
//  t_0 = PCt_0;
//  t_1 = C_0 * P[0][1];

//  P[0][0] -= K_0 * t_0;		//����������Э����
//  P[0][1] -= K_0 * t_1;
//  P[1][0] -= K_1 * t_0;
//  P[1][1] -= K_1 * t_1;
//		
//  K_Angle.X	+= K_0 * angle_er.X;			//�������	
//	K_Angle.Y	+= K_0 * angle_er.Y;			//�������
//	K_Angle.Z	+= K_0 * angle_er.Z;			//�������
//  q_bias += K_1 * angle_err;		//�������
//  angle_dot = gyro_m - q_bias;	//���ֵ��������ƣ���΢��=���ٶ�

//  *angle_f = angle;							//����Ƕ�
//  *angle_dot_f = angle_dot;			//������ٶ�
//}




//static float Q_angle = 0.001f;
//static float Q_bias = 0.003f;
//static float R_measure = 0.03f;

//float angle = 0.0f;
//float bias  = 0.0f;

//float P[2][2] = {{ 0, 0 }, { 0, 0 }};
//float KF(float newAngle, float newRate, float dt)
//{
//	/* step 1 */
//	rate 	=  newRate - bias;
//  angle += dt * rate;
//	
//	/* step 2 */
//	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
//  P[0][1] -= dt * P[1][1];
//  P[1][0] -= dt * P[1][1];
//  P[1][1] += Q_bias * dt;
//	
//	
//}


float Q_angle  =  0.01; //0.001
float Q_gyro   =  0.0003;  //0.003
float R_angle  =  0.01;  //0.03

float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float  y, S;
float K_0, K_1;

float x_angle;

// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()

float kalmanCalculate(float newAngle, float newRate,float dt)
{
	x_angle += dt * (newRate - x_bias);
	
	P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
	P_01 +=  - dt * P_11;
	P_10 +=  - dt * P_11;
	P_11 +=  + Q_gyro * dt;

	y = newAngle - x_angle;
	S = P_00 + R_angle;
	
	K_0 = P_00 / S;
	K_1 = P_10 / S;

	x_angle +=  K_0 * y;
	x_bias  +=  K_1 * y;
	
	P_00 -= K_0 * P_00;
	P_01 -= K_0 * P_01;
	P_10 -= K_1 * P_00;
	P_11 -= K_1 * P_01;

	return x_angle;
}






