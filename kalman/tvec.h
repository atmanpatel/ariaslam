//////////////////////////////////////////////////////////////////////
// tvec.h - simple STL vector class
//
// © PliaTech International 2002-2006
// Author: Michael B. Pliam
// Originated Oct 5, 2002
// Revised May 13, 2006

#pragma warning (disable: 4786)

#ifndef _TVEC_
#define _TVEC_

#include <vector>
#include <iostream>
using namespace std;

template <class T>
class tvec
{
// Attributes
private:
	vector<T> vt;		// vector of T
	long nn;			// number of elements

public :
	// Construction / Destruction
	tvec() { nn = 0; vt.resize(0); }
	tvec(long n) { nn = n; vt.resize(n); }
	tvec(vector<T> v) { nn = v.size(); vt = v; };
	tvec(long n, vector<T> v) { nn = n; vt = v; }
	tvec(long n, T x) { 
		nn = n;
		for(long i=1;i<=n;i++) 
			vt.push_back(x);
	}
	tvec(long n, T *x) { 
		nn = n; 
		for(long i=1;i<=n;i++) 
			vt.push_back(x[i-1]);
	}
	~tvec() { nn = 0; vt.resize(0); }

	long nsize() const { return nn; }

	long resize(long n) { nn = n; vt.resize(n); return n;}

	T &operator() (long i)
	{
		return vt[i - 1];

	}

};

template <class T>
ostream &operator << (ostream &s, tvec<T> &v)
{	
	cout << v.nsize() << endl;
	for(int i=1;i<=v.nsize();i++)  
		s << v(i) << endl;

	return s;

}	

#endif // _TVEC_

//////////////////////////////////////////////////////////////////////
// PliaTech 2006


// Copyright (c) 2006 by M.B. Pliam.  ALL RIGHTS RESERVED. 
// Consult your license regarding permissions and restrictions.
//

//
// This file is derived from software bearing the following
// restrictions:
//
// Copyright (c) 2006
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

