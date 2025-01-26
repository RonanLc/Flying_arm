#include "arm_math.h"
#include "sensor.h"

#ifndef __FILTERS_H
#define __FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

#define FILTER_SELECT_NONE 0
#define FILTER_SELECT_MEAN 1
#define FILTER_SELECT_KALMAN 2


/*********************************************************/
/****************** Selection du filtre ******************/
/*********************************************************/

char filter_selection = 1;
extern const double Ts ;

void Filter_init();
void Filter_selection(char new_filter_selection);
void Filter_refresh(double thrust);
double Filter_GetAngle();
double Filter_GetGyro();


/*********************************************************/
/******************** Filtre de Kalman *******************/
/*********************************************************/

#define ROW_2 2
#define COL_2 2
#define COL_1 1

// Dynamic model parameters
extern const float Inertia_Jz; 		//[m/kg]
extern const float mass_arm_prop;  	//[kg]
extern const float distOG;   		//[m]
extern const float gravity;   		//[m/s2]
extern const float L;   			//[m]
extern const float fr;    			//[N.m/s]

float32_t theta_est;								  // Position estimation container [rad]
float32_t theta_dot_est;							  // Velocity estimation container [rad/s]
float32_t theta_est_deg;							  // Position estimation container [deg]
float32_t theta_dot_est_deg;						  // Velocity estimation container [deg/s]

const float32_t Apf 			= -15.533 ;			  // - distOG * mass_arm_prop * gravity / Inertia_Jz
float32_t phi[ROW_2 * COL_2] 	= {0, 0, 0, 0};		  // Discrete-time transition state martix

float32_t X_est[ROW_2 * COL_1] 	= {0, 0};			  // State vector estimate
float32_t X_proj[ROW_2 * COL_1] = {0, 0};			  // State vector projection
float32_t Yk[ROW_2 * COL_1] 	= {0, 0};			  // Sensor data
float32_t Uk[COL_1] 			= {0};			  	  // Input vector

const float32_t var_theta 		= 0.004;			  // Variance for the potentiometer [rad]
const float32_t var_dot_theta 	= 0.004;			  // Variance for the gyrometer [rad/s]
float32_t Rk[ROW_2 * COL_2] 	= {0, 0, 0, 0};	  	  // Discrete measurement-noise covariance matrix
float32_t Hk[ROW_2 * COL_2] 	= {1, 0, 0, 1};	  	  // Sensor prediction matrix

float32_t Qk[ROW_2 * COL_2] 	= {0, 0, 0, 0};		  // Discrete-time covariance process-noise matrix
float32_t Gk[ROW_2 * COL_1] 	= {0, 0};		      // Discrete-time input transition matrix

float32_t Eye_2[ROW_2 * COL_2] 	= {1, 0, 0, 1};  	  // 2x2 eye matrix
const float32_t gam 			= 10;				  // Process' power spectral density

float32_t Pk[ROW_2 * COL_2] 	= {0, 0, 0, 0};		  // Estimation error covariance matrix
float32_t P_proj[ROW_2 * COL_2] = {10, 0, 0, 10};	  // Projection of the estimation error covariance matrix
float32_t Kk[ROW_2 * COL_2] 	= {0, 0, 0, 0};	      // Kalman Gain

// Matrix for intermediate computations
float32_t inter_1_2x2[ROW_2 * COL_2] 	= {0, 0, 0, 0};
float32_t inter_2_2x2[ROW_2 * COL_2] 	= {0, 0, 0, 0};
float32_t inter_3_2x2[ROW_2 * COL_2] 	= {0, 0, 0, 0};
float32_t inter_4_2x1[ROW_2 * COL_1] 	= {0, 0};
float32_t inter_5_2x1[ROW_2 * COL_1] 	= {0, 0};

arm_matrix_instance_f32 m_phi;
arm_matrix_instance_f32 m_X_est;
arm_matrix_instance_f32 m_X_proj;
arm_matrix_instance_f32 m_Yk;
arm_matrix_instance_f32 m_Uk;
arm_matrix_instance_f32 m_Rk;
arm_matrix_instance_f32 m_Hk;
arm_matrix_instance_f32 m_Qk;
arm_matrix_instance_f32 m_Gk;
arm_matrix_instance_f32 m_Eye_2;
arm_matrix_instance_f32 m_Pk;
arm_matrix_instance_f32 m_P_proj;
arm_matrix_instance_f32 m_Kk;

arm_matrix_instance_f32 m_inter_1_2x2;
arm_matrix_instance_f32 m_inter_2_2x2;
arm_matrix_instance_f32 m_inter_3_2x2;
arm_matrix_instance_f32 m_inter_4_2x1;
arm_matrix_instance_f32 m_inter_5_2x1;

void EKF_init();
void EKF_estimation(double thrust);
double EKF_GetAngle();
double EKF_GetGyro();


/*********************************************************/
/****************** Filtre de moyennage ******************/
/*********************************************************/

double mean_GetAngle();
double mean_GetGyro();


/*********************************************************/
/********************* Aucun filtre **********************/
/*********************************************************/

double raw_GetAngle();
double raw_GetGyro();

#ifdef __cplusplus
}
#endif

#endif /* __FILTERS_H */
