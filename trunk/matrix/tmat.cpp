//////////////////////////////////////////////////////////////////////
// tmat.cpp -  template matrix class operations
//
// copyright (c) PliaTech 2002
// originated Oct 12, 2002
// revised Nov 13, 2002

#include "tvec.h"
#include "tmat.h"

#include <math.h>

template <class T>
tmat<T> transpose(tmat<T> m)
{
	tmat<T> c(m.ncols(), m.nrows());
	for(long i=1;i<=m.ncols();i++) {
		for(long j=1;j<=m.nrows();j++) {
			c(i,j) = m(j,i);
		}
	}
	return c;

}// transpose(tmat<T> m)

template <class T>
tmat<T> add(tmat<T> a, tmat<T> b)
{
	if(a.ncols() != b.ncols() || a.nrows() != b.nrows()) {
		printf("matrices do not conform for addition\n");
		return a;
	}

	tmat<T> c(a.nrows(), a.ncols());
	for(long i=1;i<=a.nrows();i++) {
		for(long j=1;j<=a.ncols();j++) {
			c(i,j) = a(i,j) + b(i,j);
		}
	}
	return c;

}// add(tmat<T> a, tmat<T> b)

template <class T>
tmat<T> subtract(tmat<T> a, tmat<T> b)
{
	if(a.ncols() != b.ncols() || a.nrows() != b.nrows()) {
		printf("matrices do not conform for subtractionn\n");
		return a;
	}

	tmat<T> c(a.nrows(), a.ncols());
	for(long i=1;i<=a.nrows();i++) {
		for(long j=1;j<=a.ncols();j++) {
			c(i,j) = a(i,j) - b(i,j);
		}
	}
	return c;

}// subtract(tmat<T> a, tmat<T> b)

template <class T>
tmat<T> mul(tmat<T> a, tmat<T> b)
{
	if(a.ncols() != b.nrows()) {
		printf("matrices do not conform for multiplication\n");
		return a;
	}

	T sum;
	tmat<T> c(a.nrows(), b.ncols());
	for(long i=1;i<=a.nrows();i++) {
		for(long j=1;j<=b.ncols();j++) {
			sum = 0;
			for(long k=1;k<=a.ncols();k++)
				sum += a(i,k) * b(k,j);
			c(i,j) = sum;
		}
	}
	return c;
}// mul(tmat<T> a, tmat<T> b)

template <class T>
tmat<T> sweep(long k1, long k2, tmat<T> &a)
{
	double eps = 1.0e-8;
	T d;
	long i, j, k, n, it;
	
	if (a.ncols() != a.nrows())
		printf("sweep: matrix a not square\n");

	n = a.nrows();
	if (k2 < k1) { k = k1; k1 = k2; k2 = k; }

	for (k = k1; k <= k2; k++) {
		if ( fabs( a(k, k) ) < eps)
			for (it = 1; it <= n; it++)
				a(it, k) = a(k, it) = 0.0;
		else {
			d = 1.0 / a(k, k);
			a(k, k) = d;
			for (i = 1; i <= n; i++) 
				if (i != k) 
					a(i, k) *= (T) - d;
			for (j = 1; j <= n; j++) 
				if (j != k)
					a(k, j) *= (T) d; 
			for (i = 1; i <= n; i++) {
				if (i != k) {
					for (j = 1; j <= n; j++) {
						if (j != k)
							a(i, j) += a(i, k) *a(k, j) / d;
					} // end for j
				} // end for i != k
			} // end for i
		} // end else
	} // end for k
	return a;

}// sweep(long k1, long k2, tmat<T> &a)

template <class T>
tmat<T> inv(tmat<T> &a)
{
	//printf("typeid(a(1,1)).name() == %s\n", typeid(a(1,1)).name());

	if( strcmp(typeid(a(1,1)).name(), "double") != 0 && 
		strcmp(typeid(a(1,1)).name(), "float") != 0 ) {
		printf("not a floating point type\n");
		return a;
	}

	if (a.ncols() != a.nrows())
          printf("INVERSE: matrix a not square\n");

        tmat<T> b = a;
        sweep(1, b.nrows(), b);
        return b;

}   // inverse 

// 	if(strcmp(typeid(xx).name(), "long") == 0) printf("yes\n");

template <class T>
tmat<T> submatrix(tmat<T> a, long sr, long er, long sc, long ec)
// returns a designated submatrix, sa, of matrix a
{
	long i,j,ra,rs,ca,cs;
	ra = a.nrows();
	ca = a.ncols();
	if(sr < 0 || sr > ra) return a;
	if(sc < 0 || sc > ca) return a;
	rs = er - sr + 1;  
	cs = ec - sc + 1;
	tmat<T> sa(rs, cs);
	for(i=1;i<=rs;i++) {
		for(j=1;j<=cs;j++){
			sa(i,j) = a(i + rs,j + cs);
		}
	}
	return sa;
			
}// submatrix(tmat<T> a, long sr, long er, long sc, long ec)


//////////////////////////////////////////////////////////////////////
// PliaTech 2002


// Copyright (c) 2002 by M.B. Pliam.  ALL RIGHTS RESERVED. 
// Consult your license regarding permissions and restrictions.
//

//
// This file is derived from software bearing the following
// restrictions:
//
// Copyright (c) 1994
// PliaTech International
//
// Permission to use, copy, modify, distribute and sell this
// software and its documentation for any purpose is hereby
// granted without fee, provided that the above copyright notice
// appear in all copies and that both that copyright notice and
// this permission notice appear in supporting documentation.
// PliaTech International makes no representations about the
// suitability of this software for any purpose. It is provided
// "as is" without express or implied warranty.
//

