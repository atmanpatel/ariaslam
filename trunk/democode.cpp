//---libraries, defines ---------------------------------------------------------------------------
#include "Aria.h"
#include "ArLineFinder.h"
#include <math.h>
#include <iostream>
#define PI 3.14159265
#define OBSTOL 50
#define ORGTOBACK 250
#define ORGTORIGHT 165
#define DLPADDING 200
#define DWPADDING 100
#define TSDIVLANE = 2
#define TSDIVPARK = 100
#define TSTEP = 0.1

using namespace std;
#define FRONT_WALL_THRESHOLD 3000
//-------------------------------------------------------------------------------------------------

//---global variables -----------------------------------------------------------------------------
ArRobot robot;
ArSick sick;
int num_of_lines = 0;                                       // variables for lines
float X1[10],X2[10],Y1[10],Y2[10];                          // variables for lines
ArLineSegment lineset[100];                                 // variables for lines
int lineset_index = 0;                                      // variables for lines
float Xi[4]={0,0,0,0},Yi[4]={0,0,0,0},Xa=0,Ya=0,Xb=0,Yb=0;  // variables for points of interest
int nXn = 0;                                                // number of points on interest
std::vector< ArSensorReading > *readings = NULL;            // for laser readings
float parkSpotLength,parkSpotWidth;                         // parking spot parameters
ArPose parkSpotCenter;                                      // parking spot parameters
float timestep  TSTEP;                                     // getTvalue and parking parameters
float Tmin  TSTEP;                                         // getTvalue and parking parameters
float Tmax = 100;                                           // getTvalue and parking parameters
float Tstar = 0.0;                                          // getTvalue and parking parameters
float Tsdiv = 1.0;                                          // getTvalue and parking parameters
float startx = 0.0;                                         // getTvalue and parking parameters
float starty = 0.0;                                         // getTvalue and parking parameters
float starttheta = 0.0;                                     // getTvalue and parking parameters
float phimax = 0.55;                                   // getTvalue and parking parameters
float vmax = 700;                                           // getTvalue and parking parameters
float Lfb = 450.0;                                          // getTvalue and parking parameters
float Llr = 320.0;                                          // getTvalue and parking parameters
float r = 85;                                               // getTvalue and parking parameters
float dl = 100;                                             // getTvalue and parking parameters
float dw = 100;                                             // getTvalue and parking parameters
//-------------------------------------------------------------------------------------------------

bool MakingTurn = false;
//---function definitions -------------------------------------------------------------------------
void alignToWall(void);                                     // function for aligning to wall
bool checkIfCanAlign(void);                                 // function for aligning to wall
void lineFitting(void);                                     // functions for line fitting
void getLines(void);                                        // functions for line fitting
void filterNewLines(void);                                  // functions for line fitting
void mergeOldAndNewLines(void);                             // functions for line fitting
void sortLines(void);                                       // functions for line fitting
bool findPoints(void);                                      // function for finding points of interest
void calcParkSpot(void);                                    // function for calculating parking spot
void changeLanes(void);                                     // function to change lanes
void comeToStart(void);                                     // function to come to start position
void parkCar(void);                                         // function to do the actual parking
float getTValue(float,float,float,float);                   // function to get the T value
void doParking(float,float,float);                          // function to the actual motion
float getdl(float,float,float,float);                        // gets dl value from empirical formula
void MakeTurn();                                            // Make a turn to the best direction
bool TimeToMakeTurn();                                       // check if we are going to hit a wall
//-------------------------------------------------------------------------------------------------

//---main function --------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    //---variable declarations --------------------------------------------------------------------
    ArSonarDevice sonar;
    //---variable declarations complete------------------------------------------------------------
    
    //---initializations --------------------------------------------------------------------------
    Aria::init();
    ArSimpleConnector connector(&argc, argv);
    connector.parseArgs();
    if(argc>1)
    {
        connector.logOptions();
        exit(1);
    }
    robot.addRangeDevice(&sick);
    robot.addRangeDevice(&sonar);
    if(!connector.connectRobot(&robot))
    {
        printf("Could not connect to robot... exiting\n");
        Aria::shutdown();
        return 1;
    }
    robot.comInt(ArCommands::SOUNDTOG, 0);
    robot.runAsync(true);
    connector.setupLaser(&sick);
    sick.runAsync();
    if (!sick.blockingConnect())
    {
        printf("Could not connect to sick laser... exiting\n");
        Aria::shutdown();
        return 1;
    }
    robot.lock();
    robot.comInt(ArCommands::ENABLE, 1);
    robot.unlock();
    ArUtil::sleep(100);
    //---initializations complete------------------------------------------------------------------
    
    //---enter your code here----------------------------------------------------------------------
    cout << endl << endl << endl << "lineset=[];" << endl;
    robot.lock();
    robot.setVel(vmax);
    robot.unlock();
    ArUtil::sleep(1000);
    ArTime myTime;
    myTime.setToNow();
    while(1)
    {
		findPoints();
        while(myTime.mSecSince() < 250);
        alignToWall();
        while(myTime.mSecSince() < 500);
        // do lineFitting() and findPoints() every 500ms
        myTime.setToNow();
		lineFitting();
		if (TimeToMakeTurn())
		{
			printf("Time to make a turn!!\n");
			MakingTurn = true;
			MakeTurn();
			while(myTime.mSecSince() < 500);
			MakingTurn = false;			
		}        
        //cout << "%  Time taken for LineFitting = " << myTime.mSecSince() << " ms" << endl;
    }
    /*calcParkSpot();
    parkSpotLength -= 2*OBSTOL;
    parkSpotWidth -= OBSTOL;
    // account for obstacle avoidance
    Tsdiv  TSDIVLANE;            // for changing lanes
    changeLanes();
    comeToStart();
    Tsdiv  TSDIVPARK;            // for parking maneuver
    parkCar();
    */
	cout << "]; %ALL TASKS COMPLETED!!!" << endl;
    cout << " pioneerplot_new(lineset,POI,Xa,Xb,Ya,Yb,new_points) " << endl << endl;
    
	//---end of code-------------------------------------------------------------------------------
    
    //---shutdown procedure------------------------------------------------------------------------
    //sick.disconnect();
    //robot.disconnect();
    robot.lock();
    robot.setVel(0);
    robot.unlock();
    ArUtil::sleep(1000);
    Aria::shutdown();
    return 1;
}
//---end of main-----------------------------------------------------------------------------------

//---alignToWall function--------------------------------------------------------------------------
void alignToWall(void)
{
    sick.lockDevice();
    readings = sick.getRawReadingsAsVector();
    sick.unlockDevice();
    if(checkIfCanAlign())
    {
        float m = ((*readings)[155].getLocalY() - (*readings)[170].getLocalY())/((*readings)[155].getLocalX() - (*readings)[170].getLocalX());
        float theta = atan(m)*180/PI;
        //cout << "% Angle to wall = " << theta << "degrees" << endl;
        float distToWall = 0.0;
        for(int i=0;i<=15;i=i+5)
        {
            distToWall += ((*readings)[155+i].getRange())*cos((25-theta-i)*PI/180)/4.0;
        }
        if(fabs(theta)>2)
        {
            robot.setDeltaHeading(0.33*theta);
        }
        cout << "% Aligning to Wall = " << theta << " degrees" << endl;
    }
}
//---end of alignToWall function-------------------------------------------------------------------


//---checkIfCanAlign function----------------------------------------------------------------------
bool checkIfCanAlign(void)
{
    float m1 = ((*readings)[160].getLocalY() - (*readings)[170].getLocalY())/((*readings)[160].getLocalX() - (*readings)[170].getLocalX());
    float m2 = ((*readings)[150].getLocalY() - (*readings)[170].getLocalY())/((*readings)[150].getLocalX() - (*readings)[170].getLocalX());
    float m3 = ((*readings)[150].getLocalY() - (*readings)[160].getLocalY())/((*readings)[150].getLocalX() - (*readings)[160].getLocalX());
    float Ex2 = (m1*m1+m2*m2+m3*m3)/3;
    float Ex = (m1+m2+m3)/3;
    float SD = sqrt(Ex2 - Ex*Ex);
    // if the standard deviation of the slopes is very small then they belong to the same line and hence can be assumed to be the wall
    // we therefore use it for aligning ourselves
    if (SD<0.015)
    {
        return true;
    }
    else
    {
        return false;
    }
}
//---end of checkIfCanAlign function---------------------------------------------------------------

//---lineFitting function--------------------------------------------------------------------------
void lineFitting(void)
{
    getLines();
    filterNewLines();
    if(lineset_index>0)
    {
        // from the 2nd time onwards
        mergeOldAndNewLines();
    }
    sortLines();
    num_of_lines = 0;
    cout << "lineset(length(lineset)).lines=[";
    for (int i=0;i<lineset_index;i++)
    {
        cout << lineset[i].getX1() << " " << lineset[i].getY1() << " " << lineset[i].getX2() << " " << lineset[i].getY2() << endl;
    }
    cout << "];" << endl;
}
//---end of lineFitting function-------------------------------------------------------------------


//---getLines function-----------------------------------------------------------------------------
void getLines(void)
{
    ArLineFinder lineFinder(&sick);
    std::map< int, ArLineFinderSegment * > *lines = NULL;
    std::map< int, ArLineFinderSegment * >::iterator iter;
    lines = lineFinder.getLines();
    ArPose linesPose = lineFinder.getLinesTakenPose();
    cout << "lineset(length(lineset)+1).points=[" << linesPose.getX() << "," << linesPose.getY() << "," << linesPose.getTh() << "];" << endl;
    num_of_lines = 0;
    for (iter=lines->begin(); iter!=lines->end(); iter++)
    {
        if(iter->second->getY1()<0)
        {
            X1[num_of_lines] = iter->second->getX1();
            Y1[num_of_lines] = iter->second->getY1();
            X2[num_of_lines] = iter->second->getX2();
            Y2[num_of_lines++] = iter->second->getY2();
        }
    }
    
}
//---end of getLines function----------------------------------------------------------------------

//---filterNewLines function-----------------------------------------------------------------------
void filterNewLines(void)
{
    ArPose pose1s,pose1e,pose2s,pose2e;
    float dotProd[100];
    for(int i=0;i<num_of_lines-1;i++)
    {
        pose1s = ArPose(X1[i],Y1[i],0);
        pose2s = ArPose(X1[i+1],Y1[i+1],0);
        pose1e = ArPose(X2[i],Y2[i],0);
        pose2e = ArPose(X2[i+1],Y2[i+1],0);
        dotProd[i] = ((X2[i]-X1[i])*(X2[i+1]-X1[i+1]) + (Y2[i]-Y1[i])*(Y2[i+1]-Y1[i+1]));
        dotProd[i] = dotProd[i]/(pose1s.findDistanceTo(pose1e)*pose2s.findDistanceTo(pose2e));
        if((pose1e.findDistanceTo(pose2s)<100) && (dotProd[i]>0.9))
        {
            X2[i] = X2[i+1];
            Y2[i] = Y2[i+1];
            X1[i+1] = 0;
            Y1[i+1] = 0;
            X2[i+1] = 0;
            Y2[i+1] = 0;
        }
    }
}
//---end of filterNewLines function----------------------------------------------------------------

//---mergeOldAndNewLines function------------------------------------------------------------------
void mergeOldAndNewLines(void)
{
    double dotProd;
    ArPose pose1s,pose1e,pose2s,pose2e,poseEnd,poseStart;
    float d1s2s,d1e2e;
    for(int i=0;i<lineset_index;i++)
    {
        for(int j=0;j<num_of_lines;j++)
        {
            pose1s = lineset[i].getEndPoint1();
            pose1e = lineset[i].getEndPoint2();
            pose2s = ArPose(X1[j],Y1[j],0);
            pose2e = ArPose(X2[j],Y2[j],0);
            dotProd = ((pose2e.getX()-pose2s.getX())*(pose1e.getX()-pose1s.getX()) + (pose2e.getY()-pose2s.getY())*(pose1e.getY()-pose1s.getY()));
            dotProd = dotProd/(pose1s.findDistanceTo(pose1e) * pose2s.findDistanceTo(pose2e));
            if (fabs(dotProd)>0.9)
            {
                d1s2s = pose1s.findDistanceTo(pose2s);
                d1e2e = pose1e.findDistanceTo(pose2e);
                if(d1s2s<d1e2e && d1s2s<200)
                {
                    if (pose1s.getX()<pose2s.getX())
                    {                            
                        poseStart = ArPose(pose1s.getX(), pose1s.getY(), 0);
                    }
                    else
                    {                            
                        poseStart = ArPose(pose2s.getX(), pose2s.getY(), 0);
                    }
                    if (pose1s.findDistanceTo(pose1e) > pose2s.findDistanceTo(pose2e))
                        poseEnd = pose1e;
                    else
                        poseEnd = pose2e;
                    lineset[i].newEndPoints(poseStart.getX(),poseStart.getY(),poseEnd.getX(),poseEnd.getY());
                    X1[j]=0;Y1[j]=0;X2[j]=0;Y2[j]=0;
                }
                else if(d1s2s>d1e2e && d1e2e<200)
                {
                    if(pose1e.getX()>pose2e.getX())
                    {
                        poseEnd = ArPose(pose1e.getX(), pose1e.getY(), 0);
                    }
                    else
                    {
                        poseEnd = ArPose(pose2e.getX(), pose2e.getY(), 0);
                    }
                    if (pose1s.findDistanceTo(pose1e) > pose2s.findDistanceTo(pose2e))
                        poseStart = pose1s;
                    else
                        poseStart = pose2s;
                    lineset[i].newEndPoints(poseStart.getX(),poseStart.getY(),poseEnd.getX(),poseEnd.getY());
                    X1[j]=0;Y1[j]=0;X2[j]=0;Y2[j]=0;
                }
                else if(lineset[i].getDistToLine(pose2s)<200)
                {
                    poseStart = pose1s;
                    poseEnd = pose2e;
                    lineset[i].newEndPoints(poseStart.getX(),poseStart.getY(),poseEnd.getX(),poseEnd.getY());
                    X1[j]=0;Y1[j]=0;X2[j]=0;Y2[j]=0;
                }
            }
        }
    }
}
//---end of mergeOldAndNewLines function-----------------------------------------------------------

//---sortLines function----------------------------------------------------------------------------
void sortLines(void)
{
    int new_index = 0;
    int old_index = 0;
    for(int i=0;i<num_of_lines;i++)
    {
        // for everything in new set
        if (X1[i]<=0)
        {
            new_index++;
        }
        else if (X1[i]>0)
        {
            for(int j=old_index;j<lineset_index;j++)
            {
                // for everything in the old set
                if (X1[i]<(lineset[j].getX1()-50))
                {
                    // push down lineset[j:lineset_index] to lineset[j+1:lineset_index+1]
                    // insert this new line here at lineset[j]
                    for(int k=lineset_index+1;k>j;k--)
                    {
                        lineset[k] = lineset[k-1];
                    }
                    lineset_index++;
                    lineset[j] = ArLineSegment(X1[i],Y1[i],X2[i],Y2[i]);
                    old_index = j + 1;
                    new_index++;
                    break;
                }
            }
        }
    }
    for(int i=new_index;i<num_of_lines;i++)
    {
        if(X1[i]>0)
        {
            lineset[lineset_index++] = ArLineSegment(X1[i],Y1[i],X2[i],Y2[i]);
        }
    }
}
//---end of sortLines function---------------------------------------------------------------------

//---findPoints function---------------------------------------------------------------------------
bool findPoints(void)
{
    ArPose pose1s,pose1e,pose2s,pose2e;
    float m1,m2,c1,c2,dotProd,X1_new,Y1_new;
    nXn=0;
    if(lineset_index>1)
    {
        // if there are more than 1 lines , we find the points of intersection/interest
        for(int i=0;i<lineset_index-1;i++)
        {
            pose1s = lineset[i].getEndPoint1();
            pose1e = lineset[i].getEndPoint2();
            pose2s = lineset[i+1].getEndPoint1();
            pose2e = lineset[i+1].getEndPoint2();
            dotProd = ((pose2e.getX()-pose2s.getX())*(pose1e.getX()-pose1s.getX()) + (pose2e.getY()-pose2s.getY())*(pose1e.getY()-pose1s.getY()));
            dotProd = dotProd/(pose1s.findDistanceTo(pose1e) * pose2s.findDistanceTo(pose2e));
            if(fabs(dotProd)<0.4  &&  (pose2s.findDistanceTo(pose1e))<800)
            {
                // we can find the intersection point
                m1 = (pose1e.getY()-pose1s.getY())/(pose1e.getX()-pose1s.getX());
                m2 = (pose2e.getY()-pose2s.getY())/(pose2e.getX()-pose2s.getX());
                c1 = (pose1e.getX()*pose1s.getY()-pose1s.getX()*pose1e.getY())/(pose1e.getX()-pose1s.getX());
                c2 = (pose2e.getX()*pose2s.getY()-pose2s.getX()*pose2e.getY())/(pose2e.getX()-pose2s.getX());
                X1_new = -(c2-c1)/(m2-m1);
                Y1_new = m1*X1_new+c1;
                if (Xi[nXn]==0)
                {
                    // if there were no points in this place found before
                    Xi[nXn] = X1_new;
                    Yi[nXn] = Y1_new;
                }
                else
                {
                    // if there was a point in this place - average them out
                    Xi[nXn] = (Xi[nXn]+X1_new)/2;
                    Yi[nXn] = (Yi[nXn]+Y1_new)/2;
                }
                if(nXn==1)
                {
                    if(Xa==0)
                    {
                        Xa = pose2e.getX();
                        Ya = pose2e.getY();
                    }
                    else
                    {
                        Xa = (Xa + pose2e.getX())/2;
                        Ya = (Ya + pose2e.getY())/2;
                    }
                    Xb = Xi[0] + Xa - Xi[1];
                    Yb = Yi[0] + Ya - Yi[1];
                    cout << "Xa(length(lineset))=" << Xa << ";Ya(length(lineset))=" << Ya << ";" << endl;
                    cout << "Xb(length(lineset))=" << Xb << ";Yb(length(lineset))=" << Yb << ";" << endl;
                }
                cout << "POI(" << nXn+1 << ",:,length(lineset))=[" << Xi[nXn] << "," << Yi[nXn] << "];" << endl;
                nXn++;
                if (nXn==4)
                {
                    // All information obtained
                    return true;
                }
            }
        }
    }
    return false;
}
//---end of findPoints function--------------------------------------------------------------------

//---calcParkSpot function-------------------------------------------------------------------------
void calcParkSpot(void)
{
    ArPose POI[6];
    POI[0] = ArPose(Xi[0],Yi[0],0);
    POI[1] = ArPose(Xi[1],Yi[1],0);
    POI[2] = ArPose(Xa,Ya,0);
    POI[3] = ArPose(Xb,Yb,0);
    POI[4] = ArPose(Xi[2],Yi[2],0);
    POI[5] = ArPose(Xi[3],Yi[3],0);
    parkSpotLength = (POI[5].findDistanceTo(POI[2]) + POI[4].findDistanceTo(POI[3]))/2;
    parkSpotWidth = (POI[3].findDistanceTo(POI[2]) + POI[4].findDistanceTo(POI[5]))/2;
    parkSpotCenter = ArPose((Xi[2]+Xa+Xb+Xi[3])/4,(Yi[2]+Ya+Yb+Yi[3])/4,0);
    cout << "% Length = " << parkSpotLength << "  Width = " << parkSpotWidth << endl;
}
//---end of calcParkSpot function------------------------------------------------------------------

//---changeLanes function--------------------------------------------------------------------------
void changeLanes(void)
{
    ArPose currPose,targPose;
    currPose = robot.getPose();
    targPose = ArPose(Xi[3]+ORGTOBACK+40,Yi[3]+ORGTORIGHT+50,0);
    float Dl = fabs(targPose.getX() - currPose.getX());
    float Dw = fabs(targPose.getY() - currPose.getY());
    Dl = Dl - 0*DLPADDING;
    Dw = Dw - DWPADDING;
    float kphi = -1;
    float kv = +1;
    float T = getTValue(Dl,Dw,kphi,kv);
    doParking(T,kphi,kv);
}
//---end of changeLanes function-------------------------------------------------------------------

//---comeToStart function--------------------------------------------------------------------------
void comeToStart(void)
{
    ArPose currPose,targPose;
    currPose = robot.getPose();
    dw = currPose.getY()-ORGTORIGHT - Yi[3];
	cout << "% currpose.getX = " << currPose.getX() << endl;
    dl = getdl(parkSpotLength,parkSpotWidth,dw,Tsdiv);
	cout << "% dl = " << dl <<  ";" << endl;
	cout << "% startPosX = " << Xi[3]+ORGTOBACK+dl << ";" << endl;
    
	targPose = ArPose(Xi[3]+ORGTOBACK+dl+40,currPose.getY(),0);
    //robot.move(targPose.getX()-currPose.getX());
	robot.lock();
	robot.setVel(vmax);
	robot.unlock();
	ArUtil::sleep((int) ceil((targPose.getX()-currPose.getX())*10));
	robot.lock();
	robot.setVel(0);
	robot.unlock();
	cout << "% moving  " << targPose.getX()-currPose.getX() << endl;
    //while(!robot.isMoveDone());
	cout << "% startPosXreal = " << robot.getX() << ";" << endl;
    // wait until this target position is reached to start parking
}
//---end of comeToStart function-------------------------------------------------------------------

//---parkCar function------------------------------------------------------------------------------
void parkCar(void)
{
    ArPose currPose,targPose;
    currPose = robot.getPose();
    targPose = ArPose((parkSpotCenter.getX()-(parkSpotLength/2)+ORGTOBACK),(parkSpotCenter.getY()-(parkSpotWidth)+ORGTORIGHT),0);
    float Dl = fabs(targPose.getX() - currPose.getX());
    float Dw = fabs(targPose.getY() - currPose.getY());
    Dl = Dl - DLPADDING - 200 - 40;
    Dw = Dw + DWPADDING - 50;    
    float kphi = -1;
    float kv = -1;
    float T = getTValue(Dl,Dw,kphi,kv);
    doParking(T,kphi,kv);
}
//---end of parkCar function-----------------------------------------------------------------------

//---getTValue function----------------------------------------------------------------------------
float getTValue(float Dl,float  Dw,float kphi,float kv)
{
    int n = 0;
    float xdiff = 0.0;
    float ydiff = 0.0;
    float tdash = 0.0;
    float T = 0;
    while(fabs(xdiff)<fabs(Dl) && fabs(ydiff)<fabs(Dw))
    {
        T += timestep;
        if(T>Tmax)
        {
            break;
        }
        Tstar = T/Tsdiv;
        tdash = (T - Tstar)/2;
        n = (int) ceil(T/timestep)+1;
        float* timeline = new float[n];
        float* A = new float[n];
        float* B = new float[n];
        float* x = new float[n];
        float* y = new float[n];
        float* theta = new float[n];
        for(int i=0;i<n;i++)
        {
            timeline[i] = i*timestep;
            A[i] = 0.0;
            B[i] = 0.5*( 1 - cos(4*PI*timeline[i]/T) );
            x[i] = 0.0;
            y[i] = 0.0;
            theta[i] = 0.0;
            if(timeline[i]>=0 && timeline[i]<tdash)
            {
                A[i] = 1;
            }
            else if(timeline[i]>=tdash && timeline[i]<=(T-tdash))
            {
                A[i] = cos( PI*(timeline[i]-tdash)/Tstar );
            }
            else if(timeline[i]>(T-tdash) && timeline[i]<=T)
            {
                A[i] = -1;
            }
        }
        int i = 0;
        x[i] = startx;
        y[i] = starty;
        theta[i] = starttheta;
        float phi = 0.0, v = 0.0, rvel = 0.0, lvel = 0.0, dx = 0.0, dy = 0.0, dth = 0.0;
        for(i=1;i<n;i++)
        {
            phi = phimax * kphi * A[i];
            v = vmax * kv * B[i];
            rvel = v*( 2*cos(phi) + (Llr/Lfb)*sin(phi) );
            lvel = v*( 2*cos(phi) - (Llr/Lfb)*sin(phi) );
            dx = 0.5*(rvel+lvel)*cos(theta[i-1]);
            dy = 0.5*(rvel+lvel)*sin(theta[i-1]);
            dth = (rvel - lvel)/Llr;
            x[i] = x[i-1] + timestep*dx;
            y[i] = y[i-1] + timestep*dy;
            theta[i] = theta[i-1] + timestep*dth;
        }
        xdiff = x[0] - x[n-1];
        ydiff = y[0] - y[n-1];
        delete[] timeline; delete[] A; delete[] B; delete[] x; delete[] y; delete[] theta;
    }
    T -= 2*timestep;
    cout << "% Dl = " << fabs(Dl) << "  Dw = " << fabs(Dw) << "  xdiff = " << fabs(xdiff) << "  ydiff = " << fabs(ydiff) << "  T = " << T << endl;
    return T;
}
//---end of getTValue function---------------------------------------------------------------------

//---doParking function----------------------------------------------------------------------------
void doParking(float T,float kphi,float kv)
{
    Tstar = T/Tsdiv;
    float tdash = (T - Tstar)/2;
    int n = (int) ceil(T/timestep)+1;
    float* timeline = new float[n];
    float* A = new float[n];
    float* B = new float[n];
    for(int i=0;i<n;i++)
    {
        timeline[i] = i*timestep;
        A[i] = 0.0;
        B[i] = 0.5*( 1 - cos(4*PI*timeline[i]/T) );
        if(timeline[i]>=0 && timeline[i]<tdash)
        {
            A[i] = 1;
        }
        else if(timeline[i]>=tdash && timeline[i]<=(T-tdash))
        {
            A[i] = cos( PI*(timeline[i]-tdash)/Tstar );
        }
        else if(timeline[i]>(T-tdash) && timeline[i]<=T)
        {
            A[i] = -1;
        }
    }
    float phi = 0.0, v = 0.0, rvel = 0.0, lvel = 0.0;
    cout << "new_points = [" << endl;
    for(int i=0;i<n;i++)
    {
        cout << robot.getX() << "  " << robot.getY() << endl;
        phi = phimax * kphi * A[i];
        v = vmax * kv * B[i];
        rvel = v*( 2*cos(phi) + (Llr/Lfb)*sin(phi) );
        lvel = v*( 2*cos(phi) - (Llr/Lfb)*sin(phi) );
        robot.lock();
        //robot.setVel2(rvel,lvel);
        // for simulator
        robot.setVel2(lvel,rvel);
        // for robot
        robot.unlock();
        ArUtil::sleep((int)ceil(timestep*1000));
		//
		//if i = n/2, think about getting a laser reading????
    }
    robot.lock();
    robot.setVel2(0.0,0.0);
    robot.unlock();
}
//---end of doParking function---------------------------------------------------------------------

//---getdl function--------------------------------------------------------------------------------
float getdl(float L,float W,float dw_calc,float Tsdiv_calc)
{
    float x,y,y2;
    x = W + ORGTORIGHT + dw_calc;
    x = x/1000;
    // curve was fitted in metres - quadratic
    float dl_calc;
    if(Tsdiv_calc==1)
    {
        y2 = -0.9511*x*x + 4.6831*x; 
    }
    else if (Tsdiv_calc==2)
    {
        y2 = -0.9911*x*x + 3.6666*x;
    }
    else if (Tsdiv_calc==3)
    {
        y2 = -0.9988*x*x + 3.6075*x;
    }
    else if (Tsdiv_calc>3)
    {
        y2 = -x*x + 3.6*x;
    }
    y = 1000*sqrt(y2);
    dl_calc = y - L - ORGTOBACK;        
    return dl_calc;
}
//---end of getdl function-------------------------------------------------------------------------


// Time to make a turn! we are going to hit the front wall!!
bool TimeToMakeTurn()
{
	float distToFrontWall = 0;
	bool bRet = false;
	sick.lockDevice();
    readings = sick.getRawReadingsAsVector();
    sick.unlockDevice();
   
    for(int i=0;i<=20;i=i+2)
    {
        distToFrontWall += fabs(((*readings)[80+i].getRange())*sin((80+i)*PI/180));
    }
	distToFrontWall/=10;  // average distance of the front wall from the laser center!
	if (distToFrontWall < FRONT_WALL_THRESHOLD)
	{
		bRet = true;
	}
    return bRet;   
    
}

void MakeTurn()
{	
	//// find the readings which have the max value, get that angle and make a turn in 
	//// that direction.
    sick.lockDevice();
    readings = sick.getRawReadingsAsVector();
    sick.unlockDevice();
	
	double max_range = 0.0;
	double sum = 0.0;
	double max_theta = 0.0;
	// goes around the entire range, takes chunks of 10 angles and finds the 
	// avg range in tht window. We find the most promising of the theta
	// note if there are multiple interesting directions, we can actually make 
	// a decision based on the current heading and min turn angle! for later work!
	for (double theta=10.0;theta<170.0; theta+=10.0)
	{
		sum = 0.0;
		for (int i=0;i<10; i++)
		{
			sum += (*readings)[theta+i].getRange();  
		}
		sum /= 10.0;		
		if (sum > max_range)
		{
			max_range = sum;
			max_theta = theta;
		}			
	}
	printf("%Decided on a turn angle of %lf\n",(+90.0 - max_theta ));		
	
	// for 170 it shud be -90 and for 10 it shud be 90
	max_theta = (max_theta > 90.0) ? (80 - max_theta) : (100 - max_theta); 

	robot.lock();
	robot.setVel(0);
	robot.setVel2(0.0, 0.0);
	robot.unlock();

	
	robot.lock();
	robot.setDeltaHeading(double(max_theta));
	robot.unlock();

	ArUtil::sleep(1000);

	robot.lock();
	robot.setVel(vmax);
	robot.unlock();
	return;
}