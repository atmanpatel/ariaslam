#pragma warning (disable: 4786)
#ifndef RSTATE_H
#define RSTATE_H


#include "tmat.h"	// needed for determinant calculation

#include <vector>
#include <iostream>


// macro definitions 
#define PI 3.14159265


class Rstate
{

public:
	long dims;				// number of dimensions in state vector
	long numfeatures;
	tmat<double> state_X;	// state vector
	tmat<double> state_P;	// state covariance matrix

	// constructor
	Rstate(tmat<double> &X_in , tmat<double> &P_in )
	{
		state_X = X_in;
		state_P = P_in;
		dims = X_in.nrows();
		numfeatures = 0;
	}
	//augment the state vector with a new row and properly scale the P matrix
	void add_feature(long n){
		long cur_cols_x = state_X.ncols();
		long new_rows_x = dims+2*n;
		long new_cols_P = state_P.ncols()+2*n; //for P matrix
		
		tmat<double> tempx(new_rows_x,cur_cols_x,0.0);
		submatrix(tempx,1,dims,1,cur_cols_x) = state_X;
		state_X = tempx;
		
		tmat<double> tempP(new_rows_x,new_cols_P,1.0,0);
		submatrix(tempP,1,dims,1,dims) = state_P;
		state_P = tempP;

		dims = new_rows_x;
		numfeatures += n;
	}
	


	// destructor
	~Rstate()
	{
	}


};

#endif
