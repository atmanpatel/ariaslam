// xtmat.cpp  - Test Driver for tmat class
//
// MB Pliam
// Originated January 28, 2004
// Revised May 13, 2006


#include "tvec.h"	
#include "tmat.h"


//#include <crtdbg.h>

// perm: requires hconcat, flop, diagprod, delcol

tmat<int> bin_add(tmat<int> I, tmat<int> II)
{
	tmat<int> B = I + II;
	for(int i=1; i<=I.nrows(); i++) {
		for(int j=1; j<=I.ncols(); j++) {
			B(i,j) = I(i,j) + II(i,j);
			if(B(i,j) > 1) B(i,j) = 0;
		}
	}
	return B;

}// bin_add(tmat<int> I, tmat<int> II)

int main()
{
//#ifdef _DEBUG
//	int iOrg = _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG);	
//	_CrtSetDbgFlag(iOrg | _CRTDBG_LEAK_CHECK_DF);
//#endif
	
	cout << "tmat demo program" << endl;

	// build two 16 element double arrays
	double aa[] = { 1, 2, 3, 4,
				    4, 1, 2, 3,
					3, 4, 1, 2,
					2, 3, 4, 1 };

	double bb[] = { 3, 4, 5, 6,
		            6, 3, 4, 5,
					5, 6, 3, 4,
					4, 5, 6, 3 };

	// instantiate arrays as 4 x 4 matrices
	tmat<double> A(4,4,aa); cout << A << endl;
	tmat<double> B(4,4,bb); cout << B << endl;

	// check determinants
	cout << "det(A) = " << det(A) << endl;
	cout << "det(B) = " << det(B) << endl << endl;

	// exercise the matrix operators
	tmat<double> C;
	C = A + B; cout << "A + B =" << endl << C << endl;
	C = A - B; cout << "A - B =" << endl << C << endl;
	C = A * B; cout << "A * B =" << endl << C << endl;

	// exercise unary operators
	C = transpose(A); cout << "transpose(A) =" << endl << C << endl;
	C = inv(A); cout << "inv(A) =" << endl << C << endl;

	// check error flag
	C = hconcat(A, B); cout << "A | B =" << endl << C << endl;
	C = vconcat(A, B); cout << "A , B =" << endl << C << endl;
//	C = A + C; // error here
	delrow(C,5);cout << "A , B row=" << endl << C << endl;

	// string matrices
	tmat<string> S(3,3);  
	S(1,1) = "an";  S(1,2) = "to";  S(1,3) = "if";
	S(2,1) = "or";	S(2,2) = "no";	S(2,3) = "on";
	S(3,1) = "be";	S(3,2) = "am";  S(3,3) = "as";
	cout << S << endl;

	S = transpose(S); cout << S << endl;

	// integer matrices
	tmat<int> H(3,3);
	H(1,1) = 1;  H(1,2) = 0; H(1,3) = 1;
	H(2,1) = 0;  H(2,2) = 1; H(2,3) = 1;
	H(3,1) = 1;  H(3,2) = 0; H(3,3) = 0;
	cout << H << endl;

	H = bin_add(H, transpose(H));  cout << H << endl;
		


	cout << "end program" << endl;

	return 0;

}// main()