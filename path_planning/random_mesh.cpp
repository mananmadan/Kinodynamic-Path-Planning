#include<bits/stdc++.h>
#include</usr/include/opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <tr1/unordered_map>
using namespace std::tr1;
using namespace cv;
using namespace std;
#define pb push_back
int motion[8][3];
vector<vector<int> > obmap (101);
Mat img(600, 600, CV_8UC3, Scalar(0, 0, 0));//black screen

double minx,miny,maxx,maxy,xwidth,ywidth;
double max_velx=3;
double min_velx=-3;
double max_vely=3;
double min_vely=-3;
class coordinates
{
public:
  double x,y,vx,vy;
  coordinates(double x, double y,double vx,double vy)
  {
    this->x = x;
    this->y = y;
    this->vx=vx;
    this->vy=vy;
  }
};
class node
{
public:
  double x,y,cost;//gcost;
  int p_index;
  double vx,vy;

  node()
  {
	x=0;y=0;cost=0;
 vx=0;vy=0;
  }
  node(double x, double y,double vx,double vy,double cost, int p_index)
  {
    this->x = x;
    this->y = y;
    this->cost = cost;
    this->p_index=p_index;
    this->vx=vx;
    this->vy=vy;
  }

};
//bool verify_node( node node1,int minx, int miny, int maxx, int maxy);
int find_min(vector<int> obs, int n);
bool verify_node( node node1,int minx, int miny, int maxx, int maxy,node parent);
int find_max(vector<int>  obs, int n);
void calc_obstacle_map(vector<int> obsx, vector<int> obsy, int obsn, int reso, int rr);
coordinates a_star_planning(double sx, double sy,double vx,double vy,double gx, double gy,double vgx,double vgy,vector<int>  obsx, vector<int>  obsy, int obsn, int reso, int rr );
//coordinates a_star_planning(double sx, double sy, double gx, double gy, vector<int>  obsx, vector<int>  obsy, int obsn, int reso, int rr  );
void path_planning(double x_in, double y_in,double vx,double vy,double gx, double  gy,double vgx,double vgy,vector<int> ox, vector<int> oy);
void get_motion_model();
double calc_heuristic(node n1, node n2);
void revise_obstacle_coordinates(vector<int>  &ox, vector<int>  &oy);
void revise_obstacle_coordinates(vector<int>  &ox, vector<int>  &oy)
{
    int a,b;
    for (int a = 3; a <= 52; a++) {
        for (int j = 0; j < 10; j++){

            b = (rand() % (52 + 1 - 3)) + 3;
            ox.pb(a);
            oy.pb(b);
            circle(img, Point((int)a*10,(int)b*10), 5, Scalar(255, 255, 0), -1, 8, 0);
        }
    }

    return ;
}

int calc_index(node n, double xwidth, double xmin, double ymin)
{
  return (n.y - ymin)*xwidth + (n.x-xmin);
}
void path_planning(double x_in, double y_in,double vx,double vy,double gx, double  gy,double vgx,double vgy,vector<int> ox, vector<int> oy){
    /*
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    */
    int reso = 1;
    int rr = 1;
    cout<<"Starting the process"<<"\n";
    int ares_main=0;
    double rx=x_in;
    double ry=y_in;
    vector<int>opencvx;
    vector<int>opencvy;
    cout<<"Now at"<<rx<<" "<<ry<<"\n";

    cout<<"My velocity in x direction is:"<<vx<<" "<<"and in y direction is:"<<vy<<"\n";
    opencvx.pb((int)rx);
    opencvy.pb((int)ry);

    while (1)
    {
        ares_main=ares_main+1;
        coordinates ggkk(67,7,1,1);

        ggkk = a_star_planning(rx,ry,vx,vy,gx, gy,vgx,vgy,ox, oy, ox.size(),reso,rr);
        rx = ggkk.x;
        ry = ggkk.y;
       cout<<"go to ::::::"<<" ";
       cout<<rx<<" "<<ry<<"\n";

       cout<<"You will each with velcity in x:"<<ggkk.vx<<" "<<"and in y:"<<ggkk.vy<<endl;
       circle(img, Point((int)rx*10,(int)ry*10), 5, Scalar(0, 0, 255), -1, 8, 0);
       line(img,Point((opencvx[0])*10,(opencvy[0])*10),Point((int)rx*10,(int)ry*10),Scalar(255, 0, 0), 1, 8);
       imshow("Display Window",img);
       waitKey(1000);
       destroyAllWindows();
       waitKey(1);waitKey(1);waitKey(1);waitKey(1);
      opencvx[0]=(int)rx;
      opencvy[0]=(int)ry;
        if (rx==gx and ry==gy and ggkk.vx == vgx and ggkk.vy == vgy)
         {

             break;
         }


      }


    printf("Yeah! We reached the target");
    imshow("Display Window",img);
    waitKey(0);

}
coordinates a_star_planning(double sx, double sy,double vx,double vy,double gx, double gy,double vgx,double vgy,vector<int>  obsx, vector<int>  obsy, int obsn, int reso, int rr  )
{
  /*
    gx: goal x position [m]
    gx: goal x position [m]
    obsx: x position list of Obstacles [m]
    obsy: y position list of Obstacles [m]
    obsn: number of obstacles
    reso: grid resolution [m]
    rr: robot radius[m]
    */
    node nstart = node(sx/reso, sy/reso,vx,vy,0.0,-1);
    node ngoal = node(gx/reso, gy/reso,vgx,vgy,0.0, -1);

    for(int i=0;i<obsn;i++)
    {
      obsx[i] = obsx[i]/reso;
      obsy[i] = obsy[i]/reso;
    }

    calc_obstacle_map(obsx, obsy, obsn, reso, rr);
    //get_motion_model();//change this;
	//error here
    typedef tr1::unordered_map <int, node> pppp;
    pppp openset;
    pppp closedset;
    ///unordered_map <int, node> closedset;
    openset[calc_index(nstart, xwidth, minx, miny)] = nstart;
   int testing = 0;
  double finalx=0;
  double finaly=0;
  double finalvx=0;
  double finalvy=0;
  while(testing<2)
    {
      double min_temp = INT_MAX;
      int cid=0;
      for(pppp:: const_iterator i = openset.begin(); i!=openset.end(); ++i)
      {
        double rr = (i->second).cost + calc_heuristic(ngoal, i->second);
        if(rr<min_temp)
        {
          min_temp=rr;
          cid = i->first;
        }

      }
      node current = openset[cid];
     cout << current.x << "," <<  current.y << endl;
finalx=current.x;
finaly=current.y;
finalvx = current.vx;
finalvy = current.vy;
cout<<finalvx<<endl;
cout<<finalvy<<endl;

      if ( current.x == ngoal.x && current.y == ngoal.y && current.vx == ngoal.vx && current.vy == ngoal.vy)
      {
             // cout << "Goal found\n";
              ngoal.p_index = current.p_index;
              ngoal.cost = current.cost;
              break;
      }

      openset.erase(cid);
      closedset[cid] = current;

      for(int vi=min_velx;vi<=max_velx;vi++)
{      for(int vj=min_vely;vj<=max_vely;vj++)
      {
        double cost = pow((pow((vi-current.vx),2)+pow((vj-current.vy),2)),0.5);
        node dummy = node(current.x + (vi*1),current.y + (vj*1),vi,vj,cost ,cid);
	//error here xwidth intead of xw
        int n_id = calc_index(dummy, xwidth, minx, miny);

        if(closedset.find(n_id)!=closedset.end())
              continue;
        if(verify_node(dummy, minx, miny, maxx,maxy,current)==0)//have to change this.
          continue;
        if(openset.find(n_id)==openset.end())
        {
          openset[n_id] = dummy;
        }
        else
        {
          if(openset[n_id].cost>=dummy.cost)
          {
            openset[n_id] = dummy;
          }
        }

      }

}
testing++;
}
return coordinates(finalx, finaly,finalvx,finalvy);
//error here did not retunr coordinates


}

bool verify_node( node node1,int minx, int miny, int maxx, int maxy,node parent)
{
  if(node1.x<minx)
    return 0;
  else if(node1.y<miny)
    return 0;
  else if(node1.x>=maxx || node1.y>=maxy)
    return 0;
	//error here, should be node1.x-minx and same with y
  if(obmap[int(node1.x-minx)][int(node1.y-miny)])
    return 0;
  if(node1.vx>max_velx)
   return 0;
  if(node1.vx<min_velx)
   return 0;
  if(node1.vy>max_vely)
   return 0;
  if(node1.vy<min_vely)
   return 0;
//node parent = closedset[node1.p_index];
double parentx = parent.x;
double parenty = parent.y;
double currentx = node1.x;
double currenty = node1.y;
//first check if it is a straight line
if(currentx == parentx )
{
   double smally = min(currenty,parenty);
   double bigy = max(currenty,parenty);
  for(int i=smally;i<=(int)bigy;i++)
  {
    if(obmap[int(currentx-minx)][i-(int)miny])
    return 0;
  }
}
//go to check from small x co-ordinate to bigger x co-ordinate check if the robot if the calculated y point is an integer and then if it is check it there is a obstacle there.
if(currenty == parenty )
{
   double smallx = min(currentx,parentx);
   double bigx = max(currentx,parentx);
  for(int i=smallx;i<=(int)bigx;i++)
  {
    if(obmap[i-(int)minx][int(currenty-miny)])
    return 0;
  }
}
if(currenty!=parenty and currentx!=parentx)
{
  if(currentx>parentx)
  {
    for(int i=(int)parentx ;i<=(int)currentx;i++)
      {
        //calculate y;
        double y = (((int)(currentx-parentx)*(int)((double)i - parentx))/(int)(currenty-parenty))+parenty;
        int temp_y = (int)y;
        int temp_y1 = temp_y+1;
        if((temp_y-y)==0)
        {
          if(obmap[i-(int)minx][temp_y-(int)miny]||obmap[i-(int)minx][temp_y1-(int)miny])
          return 0;
        }
      }

  }
  else if(currentx<parentx)
  {
    for(int i=(int)currentx ;i<=(int)parentx;i++)
      {
        //calculate y;
        double y = (((int)(parentx-currentx)*(int)((double)i - currentx))/(int)(currenty-parenty))+parenty;
        int temp_y = (int)y;
        int temp_y1 = temp_y+1;
        if((temp_y-y)==0)
        {
          if(obmap[i-(int)minx][temp_y-(int)miny]||obmap[i-(int)minx][temp_y1-(int)miny])
          return 0;
        }
      }

  }


}
   return 1;
}



int find_min(vector<int> obs, int n)
{
  int min = obs[0];
  for(int i=1;i<n;i++)
  {
    if(min>obs[i])
    min=obs[i];
  }
  return min;
}
int find_max(vector<int> obs, int n)
{
  int max = obs[0];
  for(int i=1;i<n;i++)
  {
    if(max<obs[i])
    max=obs[i];
  }
//have to return max instead of min
  return max;
}
void calc_obstacle_map(vector<int> obsx, vector<int> obsy, int obsn, int reso, int rr)
{
  minx = 1;
  miny = 1;

  maxx = 101;
  maxy = 101;

  xwidth = maxx-minx;
  ywidth = maxy-miny;

  //size of map is 100 by 100

  for(int i=0;i<101;i++)
  {

  for(int j=0;j<=100;j++)
  obmap[i].pb(0);
  }

  for(int j=0;j<obsn;j++)
  {
      obmap[int(obsx[j]-minx)][int(obsy[j]-miny)]=1;
  }


}

void get_motion_model()
{
    // dx, dy, cost


    motion[0][0]=1;
    motion[0][1]=0;
    motion[0][2]=1;

    motion[1][0]=0;
    motion[1][1]=1;
    motion[1][2]=1;

    motion[2][0]=-1;
    motion[2][1]=0;
    motion[2][2]=1;

    motion[3][0]=0;
    motion[3][1]=-1;
    motion[3][2]=1;

    motion[4][0]=-1;
    motion[4][1]=-1;
    motion[4][2]=sqrt(2);

    motion[5][0]=-1;
    motion[5][1]=1;
    motion[5][2]=sqrt(2);

    motion[6][0]=1;
    motion[6][1]=-1;
    motion[6][2]=sqrt(2);

    motion[7][0]=1;
    motion[7][1]=1;
    motion[7][2]=sqrt(2);


}


double calc_heuristic(node n1, node n2)
{
    double w = 1.0;  //# weight of heuristic
    double d = w * sqrt((n1.x - n2.x)*(n1.x - n2.x) + (n1.y - n2.y)*(n1.y - n2.y));
    return d;
}

int main()
{
 //[10] = {1, 2, 2, 2, 2, 4, 4, 4, 4, 4}, obsyarr[10] = {6, 2, 3, 4, 6, 2, 4, 5, 6, 7};

    vector<int> ox;
    vector<int> oy;
    /*obsxarr.pb(1);obsyarr.pb(6);
    obsxarr.pb(4);obsyarr.pb(2);
    obsxarr.pb(2);obsyarr.pb(2);
    obsxarr.pb(2);obsyarr.pb(3);
    obsxarr.pb(2);obsyarr.pb(4);
    obsxarr.pb(2);obsyarr.pb(6);
    obsxarr.pb(4);obsyarr.pb(4);
    obsxarr.pb(4);obsyarr.pb(4);
    obsxarr.pb(4);obsyarr.pb(6);
    obsxarr.pb(4);obsyarr.pb(7);
*/
//making box
line(img, Point(25, 25), Point(525,25), Scalar(0, 0, 255), 1, 1);//making lines
line(img, Point(25, 25), Point(25,525), Scalar(0, 0, 255), 1, 1);//making lines
line(img, Point(25, 525), Point(525,525), Scalar(0, 0, 255), 1, 1);//making lines
line(img, Point(525, 25), Point(525,525), Scalar(0, 0, 255), 1, 1);//making lines
//making grid
for(int i=25;i<=525;i=i+10)//making y lines
{
  line(img,Point(i,25),Point(i,525),Scalar(255, 255, 255), 0, 0);
}
for(int i=25;i<=525;i=i+10)//making x lines
{
  line(img,Point(25,i),Point(525,i),Scalar(255, 255, 255), 0, 0);
}



printf("Lets start working\n");
printf("Enter starting position coordinates");
namedWindow("Display Window",WINDOW_AUTOSIZE);

double x_in, y_in,gx,gy;
cin >> x_in >> y_in;
circle(img, Point((int)x_in*10,(int)y_in*10), 5, Scalar(0, 255, 0), -1, 8, 0);
imshow("Display Window",img);
waitKey(1000);
destroyAllWindows();
waitKey(1);waitKey(1);waitKey(1);waitKey(1);
printf("Enter goal coordinates");
cin >> gx >> gy;
circle(img, Point((int)gx*10,(int)gy*10), 5, Scalar(0, 0, 255), -1, 8, 0);
imshow("Display Window",img);
waitKey(1000);
destroyAllWindows();
waitKey(1);waitKey(1);waitKey(1);waitKey(1);
double vx;
double vy;
double vgx;
double vgy;
cout<<"Enter starting x velocity"<<"\n";
cin>>vx;
cout<<"Enter starting y velocity"<<"\n";
cin>>vy;
cout<<"Enter goal x velocity"<<"\n";
cin>>vgx;
cout<<"Enter goal y velocity"<<"\n";
cin>>vgy;

revise_obstacle_coordinates(ox,oy);
path_planning(x_in, y_in,vx,vy,gx,gy,vgx,vgy,ox,oy);

}
