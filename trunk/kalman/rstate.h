#pragma warning (disable: 4786)
#ifndef RSTATE_H
#define RSTATE_H


#include "tmat.h"	// needed for determinant calculation

#include <vector>
#include <iostream>


// macro definitions 
#define PI 3.14159265
#define PMAX 10000


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
	void add_feature(double x,double y){
		long n=1;
		//calculate the size of the new state vector
		long cur_cols_x = state_X.ncols();//current length of the state vector
		long new_rows_x = dims+2*n;
		//Note: new_cols_x = cur_cols_x;
		//calculate the size of the new P matrix
		long cur_cols_P = state_P.ncols();
		long new_cols_P = cur_cols_P+2*n;
		//Note: new_rows_P = new_rows_x
		
		tmat<double> tempx(new_rows_x,cur_cols_x,0.0);
		for(long i=1;i<=dims;i++){
			for(long j=1;j<=cur_cols_x;j++){
				tempx(i,j) = state_X(i,j);
				}
			}
		tempx(dims+1,1) = x;
		tempx(dims+2,1) = x;
		//submatrix(tempx,1,dims,1,cur_cols_x) = state_X;
		state_X = tempx;
	
		tmat<double> tempP(new_rows_x,new_cols_P,PMAX,0);
		for(long i=1;i<=dims;i++){
			for(long j=1;j<=cur_cols_P;j++){
				tempP(i,j) = state_P(i,j);
			}
		}
		//submatrix(tempP,1,dims,1,dims) = state_P;
		state_P = tempP;

		dims = new_rows_x;
		numfeatures += n;
	}

	tmat<double> get_indexed_feature(long index){
		tmat<double> temp(2,1,0.0);
		//assuming index is not > numfeatures
		temp(1,1) = state_X(2*index+2,1);
		temp(2,1) = state_X(2*index+3,1);
		return temp;
		}
	
	// destructor
	~Rstate()
	{
	}


};

#endif
