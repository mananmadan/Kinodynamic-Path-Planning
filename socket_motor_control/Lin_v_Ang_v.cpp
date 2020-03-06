#include<iostream>
#include<bits/stdc++.h>
#include<cmath>
using namespace std;
double l;
double calculate_linear_velocity(double vx,double vy,double angle)
{
  double phi = angle*3.14159/180;//converting into radians
  double linear_velocity = ((vx*cos(phi)) + (vy*sin(phi)));
  return linear_velocity;
}

double calculate_angular_velocity(double vx,double vy,double angle)
{
  double phi = angle*3.14159/180;
  double angular_velocity = (((vy*cos(phi)) - (vx*sin(phi)))/(l));
  return angular_velocity;
}
