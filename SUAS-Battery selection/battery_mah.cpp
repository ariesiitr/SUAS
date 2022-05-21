#include<bits/stdc++.h>
using namespace std;
int main()
{
   // for propeller size 18*6.1 CF
   int t;
   cin >> t;
   while(t--)
   {
       float i, capacity, dsr, x, acd, tv, fot, wt,pw, noc, thrust, wattTmot;
       fot = 0.5;       // time of flight in hrs
       wt = 10;         //total all up weight in kg
       dsr = 0.8;       //80% is discharge rate
       //for 100% throttle

       thrust = 4.355;          //in kg
       wattTmot = 744.7;
       pw = wattTmot/thrust;   //power to lift 1kg wt

        cin >> noc;             //no of cells for battery
        float nv =  3.7;        //nominal voltage of a battery 
        tv =  nv*noc;           //total voltage
        i =  pw/tv;             //current
        acd = wt*i;             //calculation of Avergage current draw
        x=acd/dsr;
        capacity =  fot*x*1000;         //final capacity in mAh
        cout << capacity << endl;
   }
   return 0;
}