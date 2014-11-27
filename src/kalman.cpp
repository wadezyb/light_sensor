#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

std_msgs::Int32 average;
std_msgs::Float32 exp_pos;
std_msgs::Float32 filter_output;

// system model parameters
Matrix3f A;
RowVector3f H;
Matrix3f I;
//
Matrix3f P0;
Matrix3f Pk;
Matrix3f Pk_;
Vector3f Kk;
float R = 0.2313;
float Q = 0.00001;

// state
Vector3f xk;
Vector3f xk_;
// measurement
float zk;

float run_kalman(Vector3f xk,Matrix3f Pk, float zk)
{
  // Time update
  xk_ = A*xk;
  Pk_ = A*Pk*A.transpose();
  // Measurement update
  Kk = Pk_*H.transpose()/(H*Pk_*H.transpose()+R);
  xk = xk_ + Kk*(zk - H*xk_);
  Pk = (I - Kk*H)*Pk_;
  cout<<xk<<endl;
  return xk(0)*1000;
}

void callback( const std_msgs::Int32MultiArray::ConstPtr& msg )
{
  //ROS_INFO("JointInfo Callback");
  int i = 0;
  int temp = 0;
  float float_temp = 0;
  int counter = 0;
  float distance[]={-90,-70,-50,-30,-10,10,30,50,70,90};
  int expected_value[10];
  float expected_ratio[10];
  int white_average = 0;
  for( i= 0;i<10;i++)
    {
      temp += msg->data[i];
    }

  // Calc the average value
  average.data = temp/10;

  // expected value
  temp = 0;
  counter = 0;
  for(i=0;i<10;i++)
    {
      if( msg->data[i] < average.data )
	{
	  expected_value[i] = 0;
	  temp += msg->data[i];
	  counter++;
	} 
      else
	{
	  expected_value[i]=msg->data[i];
	}
    }
  white_average = temp/counter;
  for(i=0;i<10;i++)
    {
      if(expected_value[i]!=0) 
	{
	  expected_value[i]-=white_average;
	}
    }
  // ratio
  temp = 0;
  for(i=0;i<10;i++)
    {
      temp += expected_value[i];
    }
  for(i=0;i<10;i++)
    {
      expected_ratio[i] = (float)expected_value[i]/(float)temp;
    }
  // expected position
  float_temp = 0;
  for(i=0;i<10;i++)
    {
       float_temp += expected_ratio[i]*distance[i]; 
    }
  exp_pos.data = float_temp;
  zk = float_temp/1000;//convert from mm to m
}

int main( int argc, char **argv )
{
  A<<1,0.01,0.00005,
    0,1,0.01,
    0,0,1;
  H<<1,0,0;
  P0<<1,0,0,
    0,1,0,
    0,0,1;
  I<<1,0,0,
    0,1,0,
    0,0,1;
  xk << 0,0,0;
  Pk = P0;
  /*  */
  ros::init(argc,argv,"kalman");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("lightSensor", 10,callback);
  ros::Publisher average_pub = n.advertise<std_msgs::Int32>("average_value",5);
  ros::Publisher exp_pos_pub = n.advertise<std_msgs::Float32>("exp_pos",5);
  ros::Publisher kalman_pub = n.advertise<std_msgs::Float32>("filter_output",5);
  ros::Rate loop_rate(100);
  ROS_INFO("Run at 100Hz");
  while(ros::ok())
    {
      filter_output.data = run_kalman(xk,Pk,zk);
      //      ROS_INFO(m);
      average_pub.publish(average);
      exp_pos_pub.publish(exp_pos);
      kalman_pub.publish(filter_output);
      ros::spinOnce();
      loop_rate.sleep();
    }
  return 0;
}
