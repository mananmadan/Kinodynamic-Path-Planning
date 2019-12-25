#include<iostream>
#include<bits/stdc++.h>
using namespace std;
double width_chassi=0;
double radius_wheel=0;
double left_wheel_velocity(double linear_velocity,double angular_velocity)
{
  double numerator = (2*(linear_velocity)) + (angular_velocity*width_chassi);
  double denominator = (2*radius_wheel);
  double velocity = numerator/denominator;
  return velocity;
}

double right_wheel_velocity(double linear_velocity,double angular_velocity)
{
  double numerator = (2*(linear_velocity)) - (angular_velocity*width_chassi);
  double denominator = (2*radius_wheel);
  double velocity = numerator/denominator;
  return velocity;
}
