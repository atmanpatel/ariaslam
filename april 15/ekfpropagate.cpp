// xtmat.cpp  - Test Driver for tmat class
//
// MB Pliam
// Originated January 28, 2004
// Revised May 13, 2006

#include "tvec.h"	
#include "tmat.h"
#include "rstate.h"
#include <math.h>


// perm: requires hconcat, flop, diagprod, delcol

Rstate EKF_propagate(Rstate &state , double Vm, double Wm, double dt, double sigma_v, double sigma_w);

int main()
{
	tmat<double> A(3,3,1.0,0);
	tmat<double> x(3,1,0.0);
	
	Rstate ravi(x , A);
	//add one feature to the state vector
	ravi.add_feature(1);

	cout << ravi.state_X << endl;
	cout << ravi.state_P << endl;

	Rstate ravi1 = EKF_propagate(ravi, 10, 2, 0.1, 0.01, 0.01);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;

	
	ravi1 = EKF_propagate(ravi1, 10, 2, 0.1, 0.01, 0.01);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;

	ravi1 = EKF_propagate(ravi1, 10, 2, 0.1, 0.01, 0.01);
	cout << ravi1.state_X << endl;
	cout << ravi1.state_P << endl;

	cout << ravi1.dims << endl;
	return 0;

}

Rstate EKF_propagate(Rstate &state , double Vm, double Wm, double dt, double sigma_v, double sigma_w){
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
	cout << Q_K << endl;

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
	
	cout<<"P_RR ="<<endl<<P_RR;
	for(long i=1;i<4;i++){
		for(long j=1;j<4;j++){
			opState.state_P(i,j) = P_RR(i,j);
		}
	}
	//submatrix(opState.state_P,1,3,1,3) = P_RR;
	cout<<"P_RR updated ="<<endl<<submatrix(opState.state_P,1,3,1,3);

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
