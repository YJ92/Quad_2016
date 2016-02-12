/*
 * Kalman_Filter.c
 *
 *  Created on: 2016. 2. 12.
 *      Author: YJ
 */

#include <stdbool.h>
#include <math.h>
#include "Kalman_Filter.h"
#include "I2C_API.h"
#include "PID_v1.h"
#include "variables_map.h"

float q_kalman[4] = {1, 0, 0, 0};
float P_kalman[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
float xp[4], Pp[4][4], K[4][4], A[4][4];
float m[3] = {1,0,0};

//*****************************************************************************
// Matrix Functions
//*****************************************************************************
float determinant(float M[4][4],int k){
	float det,det00,det01,det02,det03;

	switch(k){
		case 1:
			return M[0][0];
		case 2:
			det = M[0][0]*M[1][1]-M[0][1]*M[1][0];
			return det;
		case 3:
			det=M[0][0]*(M[1][1]*M[2][2]-M[2][1]*M[1][2])+M[0][1]*(M[1][2]*M[2][0]-M[1][0]*M[2][2])+M[0][2]*(M[1][0]*M[2][1]-M[2][0]*M[1][1]);
			return det;
		case 4:
			det00=M[1][1]*(M[2][2]*M[3][3]-M[3][2]*M[2][3])+M[1][2]*(M[2][3]*M[3][1]-M[2][1]*M[3][3])+M[1][3]*(M[2][1]*M[3][2]-M[3][1]*M[2][2]);
			det01=M[1][0]*(M[2][2]*M[3][3]-M[3][2]*M[2][3])+M[1][2]*(M[2][3]*M[3][0]-M[2][0]*M[3][3])+M[1][3]*(M[2][0]*M[3][2]-M[3][0]*M[2][2]);
			det02=M[1][0]*(M[2][1]*M[3][3]-M[3][1]*M[2][3])+M[1][1]*(M[2][3]*M[3][0]-M[2][0]*M[3][3])+M[1][3]*(M[2][0]*M[3][1]-M[3][0]*M[2][1]);
			det03=M[1][0]*(M[2][1]*M[3][2]-M[3][1]*M[2][2])+M[1][1]*(M[2][2]*M[3][0]-M[2][0]*M[3][2])+M[1][2]*(M[2][0]*M[3][1]-M[3][0]*M[2][1]);

			det=M[0][0]*det00-M[0][1]*det01+M[0][2]*det02-M[0][3]*det03;
			return det;
		default:
			return 0;
	}


}

unsigned long InverseMatrix(float num[4][4],int f){

	float b[4][4]={0},fac[4][4],d;
	int p,q,m,n,i,j;

	for(q=0;q<f;q++){
		for(p=0;p<f;p++){
			m = 0; n = 0;
			for(i=0;i<f;i++){
				for(j=0;j<f;j++){
					if(i != q && j != p){
						b[m][n] = num[i][j];
						if(n<(f-2))
							n++;
						else{
							n=0;m++;
						}
					}
				}
			}
			fac[q][p] = powf(-1,p+q)*determinant(b,f-1);
		}
	}


	for(i=0;i<f;i++){
		for(j=0;j<f;j++)
			b[i][j] = fac[j][i];
	}


	d = determinant(num,f);
	if(d < 1.0e-10)
		return false;


	for(i=0;i<f;i++){
		for(j=0;j<f;j++)
			num[i][j] = b[i][j]/d;
	}

	return true;
}

//*****************************************************************************
// Calculate Euler Angle with Kalman Filter
//*****************************************************************************
void EulerAngle(float* e, float* q){
	float a[3], w[3], L[3], t[4], m_dot[3];
	float norm, inner_q;
	float q1[4], q2[4], psi;
	float buff_arr[4][4];
	int i,j,k,l;

	// 1. IMU to Quatornion
	GetFromMPU6050(a, 1);

	GetFromMPU6050(w, 0);

	norm = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
	a[0] /= norm;
	a[1] /= norm;
	a[2] /= norm;
	if(a[2] >= 0.0){
		q1[0] = sqrtf((1+a[2])/2);
		q1[1]=  a[1]/sqrtf(2*(1+a[2]));
		q1[2] = -a[0]/sqrtf(2*(1+a[2]));
		q1[3] = 0;
	}else{
		q1[0] = a[1]/sqrtf(2*(1-a[2]));
		q1[1] = sqrtf((1-a[2])/2);
		q1[2] = 0;
		q1[3] = a[0]/sqrtf(2*(1-a[2]));
	}

	// q_mag
	m_dot[0] = w[1]*m[2]-w[2]*m[1];
	m_dot[1] = w[2]*m[0]-w[0]*m[2];
	m_dot[2] = w[0]*m[1]-w[1]*m[0];

	m[0] = m[0] - m_dot[0]*dt;
	m[1] = m[1] - m_dot[1]*dt;
	m[2] = m[2] - m_dot[2]*dt;

	norm = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
	m[0] /= norm;
	m[1] /= norm;
	m[2] /= norm;

	t[0] = -q1[1]*m[0] - q1[2]*m[1] - q1[3]*m[2];
	t[1] =  q1[0]*m[0] - q1[3]*m[1] + q1[2]*m[2];
	t[2] =  q1[3]*m[0] + q1[0]*m[1] - q1[1]*m[2];
	t[3] = -q1[2]*m[0] + q1[1]*m[1] + q1[0]*m[2];

	L[0] = t[1]*q1[0] - t[0]*q1[1] + t[3]*q1[2] - t[2]*q1[3];
	L[1] = t[2]*q1[0] - t[3]*q1[1] - t[0]*q1[2] + t[1]*q1[3];
	L[2] = t[3]*q1[0] + t[2]*q1[1] - t[1]*q1[2] - t[0]*q1[3];


	psi = atan2f(L[1],L[0]);
	q2[0] = cosf(psi/2.0);
	q2[1] = 0;
	q2[2] = 0;
	q2[3] = -sinf(psi/2.0);

	// q
	q[0] = q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2] - q2[3]*q1[3];
	q[1] = q2[1]*q1[0] + q2[0]*q1[1] - q2[3]*q1[2] + q2[2]*q1[3];
	q[2] = q2[2]*q1[0] + q2[3]*q1[1] + q2[0]*q1[2] - q2[1]*q1[3];
	q[3] = q2[3]*q1[0] - q2[2]*q1[1] + q2[1]*q1[2] + q2[0]*q1[3];

	// 2. Remove quatornion discontiunity
	inner_q = q_kalman[0]*q[0] + q_kalman[1]*q[1] + q_kalman[2]*q[2] + q_kalman[3]*q[3];
	if(inner_q < 0){
		q[0] = -q[0];
		q[1] = -q[1];
		q[2] = -q[2];
		q[3] = -q[3];
	}

	// 3. Kalman Filter
		// 3.1. System Variables
	A[0][0] =          1;	A[0][1] = -dt/2*w[0];	A[0][2] = -dt/2*w[1];	A[0][3] = -dt/2*w[2];
	A[1][0] =  dt/2*w[0];	A[1][1] =          1;	A[1][2] =  dt/2*w[2];	A[1][3] = -dt/2*w[1];
	A[2][0] =  dt/2*w[1];	A[2][1] = -dt/2*w[2];	A[2][2] =          1;	A[2][3] =  dt/2*w[0];
	A[3][0] =  dt/2*w[2];	A[3][1] =  dt/2*w[1];	A[3][2] = -dt/2*w[0];	A[3][3] =          1;

		// 3.2. xp = A*x
	xp[0] = A[0][0]*q_kalman[0] + A[0][1]*q_kalman[1] + A[0][2]*q_kalman[2] + A[0][3]*q_kalman[3];
	xp[1] = A[1][0]*q_kalman[0] + A[1][1]*q_kalman[1] + A[1][2]*q_kalman[2] + A[1][3]*q_kalman[3];
	xp[2] = A[2][0]*q_kalman[0] + A[2][1]*q_kalman[1] + A[2][2]*q_kalman[2] + A[2][3]*q_kalman[3];
	xp[3] = A[3][0]*q_kalman[0] + A[3][1]*q_kalman[1] + A[3][2]*q_kalman[2] + A[3][3]*q_kalman[3];

		// 3.3. Pp = A*P*A' + Q
	for(i=0;i<4;i++){
		for(j=0;j<4;j++){
			Pp[i][j] = 0;
			for(k=0;k<4;k++){
				for(l=0;l<4;l++)
					Pp[i][j] += A[i][l]*P_kalman[l][k]*A[j][k];
			}
			if(i==j)
				Pp[i][j] += Q_q;

			buff_arr[i][j] = Pp[i][j];
		}
	}

		// 3.4. K = Pp*inv(Pp+R)
	buff_arr[0][0] += R_r;
	buff_arr[1][1] += R_r;
	buff_arr[2][2] += R_r;
	buff_arr[3][3] += R_r;

	InverseMatrix(buff_arr,4);

	for(i=0;i<4;i++){
		for(j=0;j<4;j++)
			K[i][j] = Pp[i][0]*buff_arr[0][j] + Pp[i][1]*buff_arr[1][j] + Pp[i][2]*buff_arr[2][j] + Pp[i][3]*buff_arr[3][j];
	}

		// 3.5. Update q_kalman = xp + K(q-xp)
	for(i=0;i<4;i++){
		q_kalman[i] = xp[i];
		for(j=0;j<4;j++){
			q_kalman[i] += K[i][j]*(q[j]-xp[j]);
		}
	}

		// 3.6.Update P_kalman = Pp - K*Pp
	for(i=0;i<4;i++){
		for(j=0;j<4;j++){
			P_kalman[i][j] = Pp[i][j];
			for(k=0;k<4;k++)
				P_kalman[i][j] -= K[i][k]*Pp[k][j];
		}
	}

	q[0] = q_kalman[0];
	q[1] = q_kalman[1];
	q[2] = q_kalman[2];
	q[3] = q_kalman[3];

	// 4. Euler Angle
	e[0] = atan2f(2.0*(q[2]*q[3]+q[0]*q[1]), 1.0-2.0*(q[1]*q[1]+q[2]*q[2]));
	e[1] = asinf(2.0*(q[0]*q[2]-q[3]*q[1]));
	e[2] = atan2f(2.0*(q[1]*q[2]+q[0]*q[3]), 1.0-2.0*(q[2]*q[2]+q[3]*q[3]));
}



