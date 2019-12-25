#include<iostream>
#include<bits/stdc++.h>
using namespace std;

double left_wheel_velocity(double linear_velocity,double angular_velocity,double width_chassi,double radius_wheel)
{
  double numerator = (2*(linear_velocity)) + (angular_velocity*width_chassi);
  double denominator = (2*radius_wheel);
  double velocity = numerator/denominator;
  return velocity;
}

double right_wheel_velocity(double linear_velocity,double angular_velocity,double width_chassi,double radius_wheel)
{
  double numerator = (2*(linear_velocity)) - (angular_velocity*width_chassi);
  double denominator = (2*radius_wheel);
  double velocity = numerator/denominator;
  return velocity;
}
