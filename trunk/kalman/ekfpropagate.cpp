// xtmat.cpp  - Test Driver for tmat class
//
// MB Pliam
// Originated January 28, 2004
// Revised May 13, 2006

#include "tvec.h"	
#include "tmat.h"
#include "rstate.h"
#include <math.h>

#define SIGMA_Z 0.01
// perm: requires hconcat, flop, diagprod, delcol

Rstate EKF_propagate(Rstate &state , double Vm, double Wm, double dt, double sigma_v, double sigma_w);
Rstate EKF_update(Rstate &state , tmat<double> z,bool flag,long index);

int main()
{
	tmat<double> A(3,3,1.0,0);
	tmat<double> x(3,1,0.0);
	tmat<double> z1(2,1,1.0);
	tmat<double> z2(2,1,2.0);
	tmat<double> z3(2,1,1.1);
	
	Rstate ravi(x , A);
	
	cout << ravi.state_X << endl;
	cout << ravi.state_P << endl;

	Rstate ravi1 = EKF_propagate(ravi, 10, 2, 0.1, 0.01, 0.01);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;
	//suppose we detect a new feature at z1
	//hence flag=true and index =anything(dont care)
	ravi1 = EKF_update(ravi1,z1,true,1);

	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;
	
	//propagate again
	ravi1 = EKF_propagate(ravi1, 10, 2, 0.1, 0.01, 0.01);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;

	//now suppose we see another feature at say z2
	//hence flag = true, index = dont care
	ravi1 = EKF_update(ravi1,z2,true,1);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;

	//propagate again
	ravi1 = EKF_propagate(ravi1, 10, 2, 0.1, 0.01, 0.01);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;

	//now there are two features in the state vector
	//suppose we see feature 1 again at z3
	//hence flag = false, index=1
	ravi1 = EKF_update(ravi1,z3,false,1);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;

	//propagate again
	ravi1 = EKF_propagate(ravi1, 10, 2, 0.1, 0.01, 0.01);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;

	
	return 0;

}

Rstate EKF_propagate(Rstate &state , double Vm, double Wm, double dt, double sigma_v, double sigma_w){
	printf("In EKF_propagate(state,Vm=%g,Wm=%g,dt=%g,sigma_v=%g,sigma_w=%g)\n",
		Vm,Wm,dt,sigma_v,sigma_w);
	// create the output state object
	Rstate opState = state;
	int N = opState.numfeatures; 
	
	double Phi_RR[] = {1, 0, -Vm*dt*sin(opState.state_X(3)),
				    0, 1,  Vm*dt*cos(opState.state_X(3)),
				    0, 0,  1};
	tmat<double> Phi_RR_K(3,3,Phi_RR); //3x3 Pose
	//cout<<"Phi_RR_K = "<<endl<<Phi_RR_K;
	//tmat<double> Phi_RM_K(3,2*N); //3x2N zeros
	//cout<<"Phi_RM_K = "<<endl<<Phi_RM_K;
	//tmat<double> Phi_MM_K(2*N,2*N,1.0,0); //2Nx2N identity
	//cout<<"Phi_MM_K = "<<endl<<Phi_MM_K;

	//tmat<double> Phi_K1 = hconcat(Phi_RR_K,Phi_RM_K);//first row of phi_K
	//tmat<double> Phi_K2 = hconcat(transpose(Phi_RM_K),Phi_MM_K);//second row of Phi_K
	//tmat<double> Phi_K = vconcat(Phi_K1,Phi_K2);//2N+3,2N+3 Phi_K
	
	//cout << Phi_K << endl;
	double G_R[] = {dt*sin(opState.state_X(3)), 0, 
				  dt*cos(opState.state_X(3)), 0, 
		              0,                     dt};

	tmat<double> G_R_K(3,2,G_R); 
	//tmat<double> G_M_K(2*N,2,0.0);
	//tmat<double> G_K = vconcat(G_R_K,G_M_K); //2N+3,2 G_K
	//cout << G_K << endl;
	
	double Q[] = {sigma_v*sigma_v, 0,
				  0, sigma_w*sigma_w};
	tmat<double> Q_K(2,2,Q); //2x2 Q
	//cout << Q_K << endl;

	double Jd[] = {0,-1,1,0};
	tmat<double> J(2,2,Jd);
	
	// state propagation - only the robot pose components are changed
	opState.state_X(1) = state.state_X(1) + Vm*dt*cos(state.state_X(3));
	opState.state_X(2) = state.state_X(2) + Vm*dt*sin(state.state_X(3));
	opState.state_X(3) = state.state_X(3) + Wm*dt;
	// all the map components of the state vector are unchanged


	// covariance propagation
	// just the block corresponding to the robot
	//note that the augmented P matrix with N landmarks will be 2N+3,2N+3
	tmat<double> P_RR = submatrix(state.state_P , 1, 3, 1, 3);
	P_RR = Phi_RR_K * P_RR * transpose(Phi_RR_K) + G_R_K * Q_K * transpose(G_R_K);
	
	//cout<<"P_RR ="<<endl<<P_RR;
	for(long i=1;i<4;i++){
		for(long j=1;j<4;j++){
			opState.state_P(i,j) = P_RR(i,j);
		}
	}
	//submatrix(opState.state_P,1,3,1,3) = P_RR;
	//cout<<"P_RR updated ="<<endl<<submatrix(opState.state_P,1,3,1,3);

	// if the number of dimensions > 3, then we have some landmark positions
	// in our feature vector, so we propagate their covariance as well
	if (N > 0)
	{
		tmat<double> P_RM = submatrix(state.state_P , 1, 3, 4, 2*N+3);//3,2N P_RM
		//find the product of phi_k+1*phi_k
		tmat<double> prod_phi = J*submatrix(opState.state_X,1,2,1,1);
		//construct the premultiplier for P_RM
		tmat<double> eye2(2,2,1.0,0);	
		double lr[]={0,0,1};
		tmat<double> lrow(1,3,lr);
		tmat<double> prod_phi_K = vconcat(hconcat(eye2,prod_phi),lrow);
		//find P_RM
		P_RM =  prod_phi_K* P_RM;
		//update it in state vector
		for(long i=1;i<4;i++){
			for(long j=1;j<2*N+1;j++){
				opState.state_P(i,j+3) = P_RM(i,j);
			}
		}
		//submatrix(opState.state_P,1,3,4,2*N+3) = P_RM;
		//find and update P_MR
		tmat<double> P_MR = transpose(P_RM);
		for(long i=1;i<2*N+1;i++){
			for(long j=1;j<4;j++){
				opState.state_P(i+3,j) = P_MR(i,j);
			}
		}
		
		//submatrix(opState.state_P,4,2*N+3,1,3) = P_MR;
	}
	return opState;
}


Rstate EKF_update(Rstate &state , tmat<double> z,bool flag,long index){
	printf("In EKF_update(state,z=(%g,%g),flag=%d,index=%d)\n",
		z(1,1),z(2,1),;
	
	//Initialize
	Rstate opState = state;
	long n = opState.numfeatures;
	double phi = opState.state_X(3,1);
	tmat<double> phat = submatrix(opState.state_X,1,2,1,1);
	tmat<double> plihat(2,1,0.0);
	
	double C[] = {cos(phi),-sin(phi),sin(phi),cos(phi)};
	tmat<double> Cphi(2,2,C);//the rotation matrix

	tmat<double> R(2,2,pow(SIGMA_Z,2.0),0);//measurement variance

	//build utility matrices
	double Jd[] = {0,-1,1,0};
	tmat<double> J(2,2,Jd);

	tmat<double> I22(2,2,1.0,0);//2x2 identity matrix
	tmat<double> mI22(2,2,-1.0,0); //2x2 identity matrix(-ve)
	tmat<double> o22(2,2,0.0); //2x2 zero padding
	
	//if this is a new feature
	if(flag) {
		opState.add_feature(z(1,1),z(2,1));//initialize feature
		n = opState.numfeatures;//get the incremented feature vector
		index = n; //set index to the end of the feature vector
		cout<<"Adding new feature..."<<endl;
		cout<<"Pk="<<opState.state_P;
		cout<<"xk="<<opState.state_X;
	}
	//if this is not a new feature just continue with update
	//note:assuming we are supplying a valid index
	
	plihat = submatrix(opState.state_X,2*index+2,2*index+3,1,1);
	//compute the measurement jacobian
	tmat<double> H_R = mI22*transpose(Cphi)*hconcat(I22,J*(plihat-phat));
	//cout<<"H_R="<<H_R;
	
	tmat<double> H_Li = transpose(Cphi);
	//cout<<"H_Li="<<H_Li;

	//Do not need to explicitly construct H
	//tmat<double> H=H_R;
	//note: numfeatures has already been incremented so be careful about that
	//for(long i=1;i<opState.numfeatures;i++) H = hconcat(H,o22);
	//H=hconcat(H,H_Li);
	//cout<<"H="<<H;

	//Compute residual covariance

	tmat<double> P_RR = submatrix(opState.state_P,1,3,1,3);//first 3x3 submatrix
	//cout<<"P_RR="<<P_RR;
	
	tmat<double> P_LiLi_k = submatrix(opState.state_P,2*index+2,2*index+3,2*index+2,2*index+3);//bottom right 2x2 submatrix
	//cout<<"P_LiLi_k="<<P_LiLi_k;
	
	tmat<double> S1 = H_R*P_RR*transpose(H_R); //we need this again for P_LiLi_kplus
	tmat<double> Si = S1+H_Li*P_LiLi_k*transpose(H_Li)+R;
	//cout<<"Si="<<Si;
		
	//Do not need to explicitly construct full Kalman gain
	//tmat<double> K_Li = transpose(H_Li);

	tmat<double> P_RLi_ksk = P_RR*transpose(H_R)*H_Li;
	//cout<<"P_RLi_ksk="<<P_RLi_ksk;
	tmat<double> P_LiR_ksk = transpose(P_RLi_ksk);
	tmat<double> P_LiLi_kplus = transpose(H_Li)*S1*H_Li;
	//cout<<"P_LiLi_kplus="<<P_LiLi_kplus;

	
	//update the Rstate vector with the calculated values now
	tmat<double> x_Liplus = phat+Cphi*z;
	for(long i=1;i<=2;i++) opState.state_X(2*index+i+1,1) = x_Liplus(i);

	for(long i=1;i<=3;i++)
		for(long j=1;j<=2;j++)
			opState.state_P(i,2*index+j+1) -= P_RLi_ksk(i,j);
	for(long i=1;i<=2;i++)
		for(long j=1;j<=3;j++)
			opState.state_P(2*index+i+1,j) -= P_LiR_ksk(i,j);
	for(long i=1;i<=2;i++)
		for(long j=1;j<=2;j++)
			opState.state_P(2*index+i+1,2*index+j+1) -= P_LiR_ksk(i,j);
	
	return opState;
}
