#include "filters.h"


/*********************************************************/
/****************** Selection du filtre ******************/
/*********************************************************/

/*
 * Initialise le filtre de Kalman et les capteurs
 */
void Filter_init(){

	Sensor_init();
	EKF_init();

}

/*
 * Permet de choisir quel filtre utiliser pour les données obtenues.
 */
void Filter_selection(char new_filter_selection){

	filter_selection = new_filter_selection;

}

/*
 * Permet de recalculer de nouvelles valeurs du filtre de Kalman
 */
void Filter_refresh(double thrust){

	if(filter_selection == FILTER_SELECT_KALMAN){
		EKF_estimation(thrust);
	}

}

/*
 * Renvoie une valeur d'angle de sortie des filtres
 */
double Filter_GetAngle(){
	switch (filter_selection) {
	    case FILTER_SELECT_NONE: // Raw
	    	return raw_GetAngle();
	    case FILTER_SELECT_MEAN: // Moyenne
	    	return mean_GetAngle();
	    case FILTER_SELECT_KALMAN: // Kalman
	    	return EKF_GetAngle();
	}
	return -1;
}

/*
 * Renvoie une valeur de vitesse angulaire de sortie des filtres
 */
double Filter_GetGyro(){
	switch (filter_selection) {
	    case FILTER_SELECT_NONE: // Raw
	    	return raw_GetGyro();
	    case FILTER_SELECT_MEAN: // Moyenne
	    	return mean_GetGyro();
	    case FILTER_SELECT_KALMAN: // Kalman
	    	return EKF_GetGyro();
	}
	return -1;
}

/*********************************************************/
/******************** Filtre de Kalman *******************/
/*********************************************************/

/*
 * Initializes all the matrix for the EKF.
 */
void EKF_init(){

	phi[0] = 1;
	phi[1] = Ts;
	phi[3] = 1;
	arm_mat_init_f32(&m_phi, ROW_2, COL_2, phi);

	arm_mat_init_f32(&m_X_est, ROW_2, COL_1, X_est);
	arm_mat_init_f32(&m_X_proj, ROW_2, COL_1, X_proj);
	arm_mat_init_f32(&m_Yk, ROW_2, COL_1, Yk);
	arm_mat_init_f32(&m_Uk, COL_1, COL_1, Uk);
	arm_mat_init_f32(&m_Rk, ROW_2, COL_2, Rk);
	arm_mat_init_f32(&m_Hk, ROW_2, COL_2, Hk);

	Qk[0] = gam * (Ts + pow(Ts,3)/3);
	arm_mat_init_f32(&m_Qk, ROW_2, COL_2, Qk);

	Gk[0] = Ts*Ts*L /(2*Inertia_Jz);
	Gk[1] = Ts*L/Inertia_Jz;
	arm_mat_init_f32(&m_Gk, ROW_2, COL_2, Gk);

	arm_mat_init_f32(&m_Eye_2, ROW_2, COL_2, Eye_2);
	arm_mat_init_f32(&m_Pk, ROW_2, COL_2, Pk);
	arm_mat_init_f32(&m_P_proj, ROW_2, COL_2, P_proj);
	arm_mat_init_f32(&m_Kk, ROW_2, COL_2, Kk);

	arm_mat_init_f32(&m_inter_1_2x2, ROW_2, COL_2, inter_1_2x2);
	arm_mat_init_f32(&m_inter_2_2x2, ROW_2, COL_2, inter_2_2x2);
	arm_mat_init_f32(&m_inter_3_2x2, ROW_2, COL_2, inter_3_2x2);
	arm_mat_init_f32(&m_inter_4_2x1, ROW_2, COL_1, inter_4_2x1);
	arm_mat_init_f32(&m_inter_5_2x1, ROW_2, COL_1, inter_5_2x1);
}

/*
 * Calcul une nouvelle estimation de valeur grace à l'EKF
 */
void EKF_estimation(double thrust){

	// Matrix Kk computation
	arm_mat_trans_f32(	&m_Hk,			&m_inter_1_2x2); 						// Hk^t  -> inter_1
	arm_mat_mult_f32(	&m_P_proj,		&m_inter_1_2x2,		&m_inter_2_2x2);	// P_proj * Hk^t -> inter_3
	arm_mat_mult_f32(	&m_Hk,			&m_inter_3_2x2,		&m_inter_1_2x2);	// Hk * P_proj * Hk^t -> inter_1
	arm_mat_add_f32(	&m_inter_1_2x2,	&m_Rk,				&m_inter_2_2x2);	// Hk * P_proj * Hk^t + Rk -> inter_2
	arm_mat_inverse_f32(	&m_inter_2_2x2,	&m_inter_1_2x2);					// [Hk * P_proj * Hk^t + Rk]^(-1) -> inter_1
	arm_mat_mult_f32(	&m_inter_3_2x2,	&m_inter_1_2x2,		&m_Kk);				// Kk = P_proj * Hk^t * [Hk * P_proj * Hk^t + Rk]^(-1)

	// Matrix Pk computation
	arm_mat_mult_f32(	&m_Kk,			&m_Hk,			  	&m_inter_1_2x2);	// Kk * Hk -> inter_1
	arm_mat_sub_f32(	&m_Eye_2,		&m_inter_1_2x2,		&m_inter_2_2x2);	// I2 - Kk * Hk -> inter_2
	arm_mat_mult_f32( 	&m_inter_2_2x2,	&m_P_proj,			&m_Pk);				// Pk = [I2 - Kk * Hk ] * P_proj

	// State vector estimate  X_est computation
	arm_mat_mult_f32(	&m_Hk,			&m_X_proj,			&m_inter_4_2x1);	// Hk * X_proj -> inter_4
	m_Yk.pData[0] = Sensor_GetAngleRaw();
	m_Yk.pData[1] = Sensor_GetGyroRaw();
	arm_mat_sub_f32(	&m_Yk,			&m_inter_4_2x1,		&m_inter_5_2x1); 	// Yk - Hk * X_proj -> inter_5
	arm_mat_mult_f32(	&m_Kk,			&m_inter_5_2x1,		&m_inter_4_2x1);	// Kk * [ Yk - Hk * X_proj ] -> inter_4
	arm_mat_add_f32(	&m_X_proj,		&m_inter_4_2x1,		&m_X_est);			// X_est = X_proj +  Kk * [ Yk - Hk * X_proj ]

	// Matrix phi computation
	float32_t Apf_update =  Apf * cos(m_Yk.pData[0]);
	m_phi.pData[2] = Ts * Apf_update;											// To complete the transition state matrix

	// Matrix Qk computation
	m_Qk.pData[1] = Ts * Ts * (Apf_update + 1) * 0.5;
	m_Qk.pData[2] = m_Qk.pData[1];
	m_Qk.pData[3] = pow(Ts,3) * Apf_update * Apf_update / 3 + Ts;

	// State vector projection computation
	arm_mat_mult_f32(	&m_phi,			&m_X_est,			&m_inter_1_2x2);	// phi * X_est -> inter_1
	m_Uk.pData[0] = thrust;
	arm_mat_mult_f32(	&m_Gk,			&m_Uk,				&m_inter_2_2x2); 	// Gk * Uk -> inter_2
	arm_mat_add_f32( 	&m_inter_1_2x2,	&m_inter_2_2x2,		&m_inter_3_2x2);	// phi * X_est + Gk * Uk -> inter_3
	m_inter_3_2x2.pData[0] = Ts * m_inter_3_2x2.pData[0];
	m_inter_3_2x2.pData[1] = Ts * m_inter_3_2x2.pData[1];
	m_inter_3_2x2.pData[2] = Ts * m_inter_3_2x2.pData[2];
	m_inter_3_2x2.pData[3] = Ts * m_inter_3_2x2.pData[3];						// Not a very efficient way of doing this, I agree.
	arm_mat_add_f32(	&m_X_est,		&m_inter_3_2x2,		&m_X_proj);			// X_proj = X_est + Ts * [ phi * X_est + Gk * Uk ]

	// Matrix P_proj computation
	arm_mat_trans_f32(	&m_phi,			&m_inter_1_2x2);						// phi^t -> inter_1
	arm_mat_mult_f32(	&m_Pk,			&m_inter_1_2x2,		&m_inter_2_2x2);	// Pk * phi^t -> inter_2
	arm_mat_mult_f32(	&m_phi,			&m_inter_2_2x2,		&m_inter_1_2x2);	// phi * Pk *phi^t -> inter_1
	arm_mat_add_f32(	&m_inter_1_2x2,	&m_Qk,				&m_P_proj);			// P_proj = phi * Pk *phi^t + Qk

	// Return the estimated data
	//return m_X_est.pData;
}


/*
 * Return the last Angle value from the EKF.
 */
double EKF_GetAngle(){

	return (double)m_X_est.pData[0];

}


/*
 * Return the last gyro value from the EKF.
 */
double EKF_GetGyro(){

	return (double)m_X_est.pData[1];

}





/*********************************************************/
/****************** Filtre de moyennage ******************/
/*********************************************************/


/*
 * Return the last gyro value from the mean filter.
 */
double mean_GetAngle(){

	return Sensor_GetAngle();

}


/*
 * Return the last gyro value from the mean filter.
 */
double mean_GetGyro(){

	return Sensor_GetGyro();

}



/*********************************************************/
/********************* Aucun filtre **********************/
/*********************************************************/


/*
 * Return the last angle value from the DMA.
 */
double raw_GetAngle(){

	return Sensor_GetAngleRaw();

}


/*
 * Return the last gyro value from the DMA.
 */
double raw_GetGyro(){

	return Sensor_GetGyroRaw();

}


