TMAT - A Simple STL Matrix Class

The tmat class uses the template class tvec which uses the STL 
vector class.  Both tmat and tvec have the commonly used vector 
and matrix operators built in.  Both classes are expandible, that 
is, you can write your own extensions of these classes using both 
template and non-template frameworks.

At the present time, there is no wrapper class for the template 
classes to allow them to be incorporated into a DLL.  

The existing error checking is pretty basic and needs to be enhanced
for rigorous use.  Error messages are simply pumped through the
CError member function of tmat using the printf statement.  This
error handling can be easily changed for more elaborate environments
such as Windows, simply by substituting MessageBox for the error
output and using more advanced error handling methods such as try
and catch with throw exception.

While there are many matrix classes available, this one is soley
based on STL.  It is being made freely available because of the
frequent requests for an STL matrix class, and in the hope that
users of the source code will further develop it and provide us
with feedback.

This project will compile and run in Visual C++.NET (VC7.0) and
in Visual C++ 6.0.  To use the class in another application, simply
add the tvec.h and tmat.h files to your project and include their
names in some appropriate place in your app source code.  For use
with the C++.NET compiler, you will need to change the application
properties under the C/C++ page Create/Use Precompiled Header to
Automatically Generate(/YX).

This code is provided 'as is'.  If you decide to use it, you do so
soley at your own risk.  The author and PliaTech Interational 
will not be responsible for any losses of time, money, equipment,
hair, health, or general psychological well-being suffered through
the use of this code.

Please report any bugs, criticism, comments, or other communications
regarding this project to:

PliaTech International:  http://www.pliatech.com  'contact us'.

Thank you for your interest in this project.

Note: This project has been recently updated to reflect changes suggest
by users of the classes.

Michael B. Pliam
PliaTech International
May 13, 2006
