#include "ros/ros.h"
#include "ros/package.h"

#include <thread>
#include <complex>
#include <valarray>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <string>
#include <memory>
#include <stdio.h>
#include <signal.h>
#include <algorithm>
#include <math.h>
#include <vector>
#include <ctime>
#include <chrono>
#include <math.h>
//#include "vector2d.hpp"

#include <boost/functional/hash.hpp>

#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Point.h"
#include "tes_vrep/Person.h"

#include <opencv/cv.h>
#include <opencv/cv.hpp>
//#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>

#define RAD2DEG(x) ((x)*180./M_PI)

#define tes 1000
#define pi 3.14159265
#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )

using namespace std;
using namespace cv;

int flagmove=0;
float Fh, Fhsoc, Fhphy;float fsoc_t;

#define imgArr 10
//#define centerX 400
//#define centerY 400
#define centerX 100
#define centerY 200
//#define scaleFactor 50
#define scaleFactor 40 //40
#define iter	200


float x_now, y_now, t_now;
float x_last, y_last, t_last;
float enc_xL, enc_yL, enc_tL;
float enc_x, enc_y, enc_t;
float x_pos,y_pos,t_pos,ti_pos,t_proc;

//int loop=0;
float datLaser[3][2][2000];
float laserData[2000];
float angle_step;
unsigned int scan_size;
bool scanready=false;
float xtemp=0,ytemp=0,ttemp=0;
float ddx, ddy;
float x_coo, y_coo;
Mat img = Mat(670, 670, CV_8UC3, cv::Scalar(0));
Mat imgSave[imgArr]; 
unsigned long int p=0;
float degx,degy,dx,dy;
float dr_x,dr_y;

float min_dist_lid_n, min_dist_lid;

fstream robot_vel;
fstream object_rel_dist;
fstream person_rel_dist;

int n_sdt_scan = 0;
float total_sdt_err = 0;
float alpa,gyro=0,gyroo,gyrooo=-89.577;
float robotgyro_z;

void pidbenda(int xTarget,int yTarget,int tTarget);
ros::Publisher motor1pub;
ros::Publisher motor2pub;
ros::Publisher motor3pub;
ros::Publisher motor4pub;

ros::Subscriber obj1_pos_x;
ros::Subscriber obj1_pos_y;
ros::Subscriber obj1_pos_z;
ros::Subscriber obj1_pos_t;

std_msgs::Float32 motor1_msg;
std_msgs::Float32 motor2_msg;
std_msgs::Float32 motor3_msg;
std_msgs::Float32 motor4_msg;

ros::Subscriber LaserScan;

//float goal[1][3] = {{5, 1.5, 90}};
float goal[3][3] = {{5, 1.5, 90}, {5, 1.5, 90}, {5, 1.5, 90}};
float sudut_object1;

bool target_detect = false;
bool object_detect = false;
bool person_detect = false;
bool person_detect1 = false;

struct object_position
{
	float pos_x;
	float pos_y;
	float pos_z;
	float pos_t;

	float new_pos_x, new_pos_y, new_pos_z;
	float new_pos_t;

	float m_pos_x, m_pos_y, m_pos_z;
	float m_pos_t;

	float dist_x;
	float dist_y;
	float distance;

	float theta;
	float theta_test;

	float error,errorTheta;
	float Derror;
	float Ierror;
	float proportional,proportionalTheta;
	float derivative,derivativeTheta;
	float integral,integralTheta;
	float sumError,sumErrorTheta;
	float lasterror,lasterrorTheta;
	float speed;
	float errteta;

	float speedTheta;
	float speedx,speedy,speedt;
	float forcealpha;

    float fs_phy, fs_soc;

	float rep_static, ep_dynamic;
	float spd_rep_static, spd_rep_dynamic;
	float radius_robot, radius_human;
	float rr, rk, rh;				//radius robot, objek
	float psi_h, psi_k;				//jarak efektif gaya (proxemics distance)
	float k_h, k_k;					//magnitude of force
	float d_rh, d_rk;				//jarak robot & objek
	float e_rh, e_rk;				//arah vektor
	float omega_k;                 //anisotrophic factor -> ω = λ + 0.5(1 − λ) (1 + cos(φ))
	float lambda_isotropy;
};
struct object_position object1;

struct object_position1
{
	float post_x;
	float post_y;
	float post_t;

	float dist_x;
	float dist_y;
	float distance;

	float theta;
	float theta_test;

	float error,errorTheta;
	float Derror;
	float Ierror;
	float proportional,proportionalTheta;
	float derivative,derivativeTheta;
	float integral,integralTheta;
	float sumError,sumErrorTheta;
	float lasterror,lasterrorTheta;
	float speed;
	float errteta;

	float speedTheta;
	float speedx,speedy,speedt;
	float forcealpha;

    float fs_phy, fs_soc;

	float rep_static, ep_dynamic;
	float spd_rep_static, spd_rep_dynamic;
	float radius_robot, radius_human;
	float rr, rk, rh;				//radius robot, objek
	float psi_h, psi_k;				//jarak efektif gaya (proxemics distance)
	float k_h, k_k;					//magnitude of force
	float d_rh, d_rk;				//jarak robot & objek
	float e_rh, e_rk;				//arah vektor
    float omega_k;                 //anisotrophic factor -> ω = λ + 0.5(1 − λ) (1 + cos(φ))
};
struct object_position object2;



struct person_position{
	float posx;
	float posy;
	float post_z;
	float post;

	float posxx; //person 1
	float posyy;
	float possxx; //person 2
	float possyy;

	float posxxx;
	float posyyy;

	float odomxx; //odom person 1
	float odomyy;
	float odommxx; //odom person 2
	float odommyy;

	float new_pos_x, new_pos_y, new_pos_z;
	float new_pos_t;

	float posx_msg, posy_msg;

	float dist_x;
	float dist_y;
	float distance;

	float theta;
	float theta_test;

	float error,errorTheta;
	float Derror;
	float Ierror;
	float proportional,proportionalTheta;
	float derivative,derivativeTheta;
	float integral,integralTheta;
	float sumError,sumErrorTheta;
	float lasterror,lasterrorTheta;
	float speed;
	float errteta;

	float speedTheta;
	float speedx,speedy,speedt;
	float speedxx,speedyy,speedtt;
	float forcealpha;

    float fs_phy, fs_soc;

	float rep_static, ep_dynamic;
	float spd_rep_static, spd_rep_dynamic;
	float radius_robot, radius_human;
	
	float rr, rk, rh;				//radius robot, objek
	float psi_h, psi_k;				//jarak efektif gaya (proxemics distance)
	float k_h, k_k;					//magnitude of force
	float kh_phy, kh_soc;			//magnitude of force
	float d_rh, d_rk;				//jarak robot & objek
	float e_rh, e_rk;				//arah vektor
	float omega_k;               	//anisotrophic factor -> ω = λ + 0.5(1 − λ) (1 + cos(φ))
	float lambda_isotropy;

	float rep_force, spd_rep_force;

	float fin_posx, fin_posy;

	float last_odom_x, last_odom_y;

};
struct person_position person;




float speedMotor1,speedMotor2,speedMotor3,speedMotor4;

struct varRobot{
	float Motor1;
	float Motor2;
	float Motor3;
	float Motor4;
	
	float posx;
	float posy;
	float post;
	float posxx;
	float posyy;
	float new_posx, new_posy;
	float new_post;

	float odom_x;
	float odom_y;
	float odom_t;

	float imux_dat;
	float imuy_dat;
	float imuz_dat;
	float imuzs_dat;

	float gyro_x, gyro_y, gyro_z;
	float new_gyro_z;
	//float gyrooo=-89.577;
	
	float velocity;
	float d_velocity;
	float l_odomx, l_odomy, l_odomt;
	float l_posx, l_posy, l_post;
	float l_gyroz;

	float rR;

	float line_velo_x, line_velo_y, line_velo;
	
	CvPoint2D32f p_velocity;
};
struct varRobot robot,friends;

struct varObs{
	float posx;
	float posy;
	float post;
};
struct varObs obstacle[6];

struct sPIDtarget 
{
	float error,errorTheta;
	float Derror;
	float Ierror;
	float proportional,proportionalTheta;
	float derivative,derivativeTheta;
	float integral,integralTheta;
	float sumError;
	float sumErrorTheta=0;
	float lasterror;
	float lasterrorTheta=0;
	float speed,kecepatan;
	float errteta;

	float distanceX;
	float distanceY;
	float distance;

	float posx;
	float posy;
	float post;
	float theta;
	float speedTheta;
	float speedx,speedy,speedt;
	float forcealpha;

	float desired_speed;
	float f_goal;
	float T = 0.54;
	int goal_counts = 0;
	float gaya_navigasi;
};
struct sPIDtarget target,targetf; 

struct sFuzzy 
{
	float distoObsx;
	float distoObsy;
	float distoObs;
};
struct sFuzzy dis;

struct sPoint
	{
		float x;
		float y;
		//float z;
	};
sPoint Point_s,substarget;


char step = 1;
void pidTarget(int xTarget,int yTarget,int tTarget);

double getDegree(int x,int y){
double deg;
	deg=atan2((double)x,(double)y)*180/M_PI;
	return deg;
}

float getDistance(float x0, float y0, float x1, float y1){
	float distanceX, distanceY;
	float distance;

	distanceX = x1 - x0;
	distanceY = y1 - y0;
	distance  = (float)(sqrt(distanceX*distanceX + distanceY*distanceY)); 

	return distance;
}

char ts;
void  timeSamplingCallback(const ros::TimerEvent&)
{
   ts=1;
}

void motoinAct(float speed1,float speed2,float speed3,float speed4){
	    motor1_msg.data = speed1;
		motor2_msg.data = speed2;
		motor3_msg.data = speed3;
		motor4_msg.data = speed4;
		
		motor1pub.publish(motor1_msg);
		motor2pub.publish(motor2_msg);
		motor3pub.publish(motor3_msg);
		motor4pub.publish(motor4_msg);
}

void mySigintHandler(int sig){
	
	for(int i=0;i<=5; i++){
	speedMotor1=0;speedMotor2=0;speedMotor3=0;speedMotor4=0;

	motoinAct(speedMotor1,speedMotor2,speedMotor3,speedMotor4);
	ROS_WARN("motor1 = %.3f | motor2 = %.3f | motor3 = %.3f | motor4 = %.3f",speedMotor1,speedMotor2,speedMotor3,speedMotor4);
	
	}
	ros::shutdown();
}


void robotgerak_c(float velX, float velY, float velW){		/// semua dalam cm (scaleFactor)
	//Refference from Al-kwarizmi omni directional
	//Jarak roda ke pusat = 27.7350 cm 
	//VelX = w * r;//VelY = w * r; //VelW = w * r;//	r(cm)
	
	velX = velX/scaleFactor;
	velY = velY/scaleFactor;
	//velW = velW/scaleFactor;
	
	printf("velx = %.3f, vely = %.3f, velw = %.3f\n",velX,velY,velW);
	
	//float L2 = 25.5538;
	//float L1 = 12.7769;
	float L2 =1;
	float L1 =0;

	//printf("velXYW = %f %f %f\n",velX,velY,velW);
	//const float wheelRadius = 0.05 ; //5CM//0.05 meter
	const float wheelRadius = 1;
	/*
	//INVERS KINEMATIC 
	robot.Motor1 = (1*velX - 1*velY - ((L1+L2))*velW); //Notation m/s
	robot.Motor2 = (1*velX + 1*velY + ((L1+L2))*velW); //??
	robot.Motor3 = (1*velX + 1*velY - ((L1+L2))*velW);
	robot.Motor4 = (1*velX - 1*velY + ((L1+L2))*velW);
	*/
	
	//INVERS KINEMATIC 
	robot.Motor1 = (1*velX - 1*velY - ((L1+L2)/100)*velW); //Notation m/s
	robot.Motor2 = (1*velX + 1*velY + ((L1+L2)/100)*velW); //??
	robot.Motor3 = (1*velX + 1*velY - ((L1+L2)/100)*velW);
	robot.Motor4 = (1*velX - 1*velY + ((L1+L2)/100)*velW);
	//Convert from m/s to RPM
	robot.Motor1 = ((robot.Motor1) / (wheelRadius/100)); //Notation Rad/s
	robot.Motor2 = ((robot.Motor2) / (wheelRadius/100)); //??
	robot.Motor3 = ((robot.Motor3) / (wheelRadius/100));
	robot.Motor4 = ((robot.Motor4) / (wheelRadius/100));
	
	/*robot.Motor1 = 4.378000; 
	robot.Motor2 = 34.842003;
	robot.Motor3 = 12.222001;
	robot.Motor4 = 26.998001;
	*/
	printf("robot.Motor = %f %f %f %f\n",robot.Motor1,robot.Motor2,robot.Motor3,robot.Motor4);
	motoinAct(robot.Motor1,robot.Motor2,robot.Motor3,robot.Motor4);
}


/*
void SF_goal_1(float xTarget, float yTarget, float tTarget){
	const float T_S = 0.05;
	float target_radius = 10;
	float d_velo = 0.8;		//desired velocity
	float r_velo; 			//velocity saat ini
	float robot_distance;
	float robot_disx, robot_disy;


	target.desired_speed = 40;

	robot.new_posx = centerX + (robot.odom_x * scaleFactor);
	robot.new_posy = centerY - (robot.odom_y * scaleFactor);

	target.posx = xTarget;
	target.posy = yTarget;
	target.post = tTarget;

	printf("robot.new_posx = %.3f, posy = %.3f\n",robot.new_posx,robot.new_posy);

	target.distanceX = target.posx - robot.new_posx;
	target.distanceY = - target.posy + robot.new_posy;
	target.distance = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta = 	(float) getDegree(target.distanceX,target.distanceY);

	printf("target.distance = %.3f\n", target.distance);
	printf("target.theta = %.3f\n", target.theta);


	//target.forcealpha = ((target.desired_speed * target.distance) - target.distance ) * (1/T_S);
	
	target.forcealpha = (target.desired_speed - robot.line_velo) / T_S;
	printf("diff velocity = %.3f\n",(target.desired_speed - robot.line_velo));
	printf("target forcealpha = %.3f\n", target.forcealpha);

	robot_vel.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/vt_robot.txt", ios::in | ios::out | ios::app);
	robot_vel << robot.line_velo/scaleFactor << endl;
	robot_vel.close();

	
	target.error = target.forcealpha;
	//target.speed = target.forcealpha;
	//==========PID TARGET============//

	target.proportional = 0.105* target.error;
	target.derivative   = 0.0025*(target.error - target.lasterror)/T_S;
	target.integral     = 0.0001*target.sumError*T_S;
	target.lasterror    = target.error;
	target.sumError    += target.error;

	
	if(target.sumError>4000){
		target.sumError=4000;}	//max force??
	else if(target.sumError<-4000){
		target.sumError=-4000;}	//min force??
	target.speed=target.proportional+target.derivative+target.integral;
	//=================END==================// 
	//target.speed = 0;
	printf("trgt spd = %.3f\n", target.speed);
//	target.speed = target.forcealpha;

	//==========PID Heading============//
	//Input 0<>179 ~ -1<>-180///CW Imu negative
	//need to look the result from input
	printf("robot.gyro_x = %.3f, y = %.3f, z = %.3f\n",robot.gyro_x,robot.gyro_y,robot.gyro_z);
	gyro = 360 - robot.gyro_z;
	printf("gyro = %.3f\n",gyro);
	
	gyroo = robot.post - gyro;
	printf("gyroo = %.3f\n",gyroo);
	
	target.errorTheta        = (-target.post) - gyroo;	
	printf("robot.post = %.3f\n", robot.post);
	printf("target.errorTheta = %.3f\n", target.errorTheta);
	if(target.errorTheta >180){
		target.errorTheta -= 360;}
	else if(target.errorTheta <-180){
		target.errorTheta += 360;}
	printf("errortheta normal = %f \n ",target.errorTheta );


	n_sdt_scan++;
	total_sdt_err = total_sdt_err + abs(target.errorTheta);

	printf("n_sdt_scan = %d \n ",n_sdt_scan );
	printf("total_sdt_err = %.3f \n ",total_sdt_err );


	//target.sumErrorTheta    += target.errorTheta*T_S;
	//if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
	//else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
	
	//target.proportionalTheta = 0.514*target.errorTheta;
	target.proportionalTheta = 1.86*target.errorTheta;
	//target.derivativeTheta   = 4.5*(target.errorTheta-target.lasterrorTheta)/T_S;
	//target.derivativeTheta   = 0.008*(target.errorTheta-target.lasterrorTheta)/T_S; //*100/5
	target.derivativeTheta   = 0.008*(target.errorTheta-target.lasterrorTheta)/T_S;
	//target.integralTheta	 = 0.7*target.sumErrorTheta;
	//target.integralTheta	 = 0.01*target.sumErrorTheta;
	//target.integralTheta	 = 0.007*target.sumErrorTheta;
	target.integralTheta	 = 0.01*target.sumErrorTheta;
	target.lasterrorTheta    = target.errorTheta;
	target.speedTheta 		 = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
	
	target.sumErrorTheta    += target.errorTheta;
	if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
	else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
	
	target.speedTheta = 1*target.speedTheta;
	//============END=================//
	//}
	printf("target.speedTheta = %.3f\n", target.speedTheta);
	
	//============Convert theta from worldFrame To robotFrame=========//
	target.errteta = target.theta + gyroo;	
	printf("target.errteta = %.3f\n", target.errteta);

	if(target.errteta > 180 ){ 
		target.errteta -= 360;}
	else if(target.errteta < -180 ){ 
		target.errteta += 360;}
	printf("target.errteta normal = %.3f\n", target.errteta);
	//==============END Convert theta from worldFrame To robotFrame===//
	target.speedx = target.speed * sin(target.errteta/57.2957795);
	target.speedy = target.speed * cos(target.errteta/57.2957795);
	target.speedt = target.speedTheta;


	printf("trgt speedx = %.3f\n", target.speedx);
	printf("trgt speedy = %.3f\n", target.speedy);

	//target.speedx = target.speedx + object1.speedx + person.speedx;
	//target.speedy = target.speedy + object1.speedy + person.speedy;
	//target.speedt = target.speedt;



	//printf("robot speedx = %.3f\n", target.speedx);
	//printf("robot speedy = %.3f\n", target.speedy);
	

	if(target.distance > target_radius){
		robotgerak_c(target.speedx ,target.speedy,target.speedt);
		target_detect = true;}
	else{ 
		//if(target.goal_counts < 2){
		//	target.goal_counts++;
		//}
		robotgerak_c(0,0,0); 
	}  
}
*/

//////////////////////////// VERSION 3 ///////////////////////
void SF_static_obj_3(){
	const float T_S = 0.05; //0.05
	float target_radius = 5;
	float rR = 40; 			//radius robot
	float d_velo = 0.8;		//desired velocity
	float r_velo; 			//velocity saat ini
	float robot_distance;
	float robot_disx, robot_disy;
	float abssudut_object1,jarak;

	object1.k_h = 50; //7.5 //Magnitude Force ///////65.5
	//object1.k_h = 25.5;

	target.desired_speed = 40;

	object1.lambda_isotropy = 1;
	float SIGMA_PHYSICAL = 4; //Parameter describing the range scale of the physical Force
	/*
	""" Obstacle Forces"""
	# Parameter describing the magnitude of the force obstacle force.
	F_OBSTACLE = 20.5
	# Parameter describing the range scale of the obstacle Force.
	SIGMA_OBSTACLE = 0.2
	""""""""""""""""""""
	*/

	///////////////		Position of closes object	/////////////////////////
	float ldegx, ldegy, ldx, ldy, lx_coo, ly_coo;
	float langle = (float)270 / (float)scan_size;

	ldegx = cos(((langle*(min_dist_lid_n))-45) * M_PI / 180); //rad
	ldegy = sin(((langle*(min_dist_lid_n))-45) * M_PI / 180);

	ldx = (min_dist_lid * scaleFactor)*ldegx;	//rad * distance	//(0.5*distance) * cos(((angle_step*(i))-45) * M_PI / 180)
	ldy = (min_dist_lid * scaleFactor)*ldegy;

	lx_coo = (cos(robot.new_gyro_z) * (ldx - 0)) - (sin(robot.new_gyro_z) * (ldy - 0)) + (centerX + (robot.odom_x * scaleFactor));
	ly_coo = (sin(robot.new_gyro_z) * (ldx - 0)) + (cos(robot.new_gyro_z) * (ldy - 0)) - (centerY - (robot.odom_y * scaleFactor));

	ly_coo = -ly_coo;

	object1.m_pos_x = lx_coo;
	object1.m_pos_y = ly_coo;
	printf("object1.m_pos_x = %.3f, y = %.3f\n",object1.m_pos_x,object1.m_pos_y);

	robot.new_posx = centerX + (robot.odom_x * scaleFactor);
	robot.new_posy = centerY - (robot.odom_y * scaleFactor);

	//object1.posx & object1.posy didapat dari LIDAR
	object1.dist_x = object1.m_pos_x - robot.new_posx;
	object1.dist_y =  -object1.m_pos_y + robot.new_posy;
	printf("object1.dist_x = %.3f, y = %.3f\n",object1.dist_x,object1.dist_y);

	object1.distance = (sqrt((double)object1.dist_x*(double)object1.dist_x + (double)object1.dist_y*(double)object1.dist_y));
	object1.theta = (float)getDegree(object1.dist_x,object1.dist_y);
	printf("object1.distance = %.3f\n", object1.distance);
	printf("object1.theta = %.3f\n", object1.theta);

	sudut_object1 = object1.theta-90;
	if(object1.theta<-90)
	{
		sudut_object1 = abs(sudut_object1)-180;
		sudut_object1 = 180-sudut_object1;
	}
	printf("sudut_object1 = %.3f\n",sudut_object1);

	object1.fs_soc = abs(sudut_object1)*object1.k_h * exp((rR - object1.distance)/SIGMA_PHYSICAL);
	object1.fs_phy = abs(sudut_object1)*object1.k_h * (rR - object1.distance);
	abssudut_object1 = abs(sudut_object1);
	jarak = rR - object1.distance;
	printf("object1.fs_soc = %.3f, fs_phy = %.3f\n",object1.fs_soc,object1.fs_phy);
	printf("abssudut = %.3f, jarak = %.3f\n",abssudut_object1,jarak);

	

	object1.rep_static = object1.fs_phy + object1.fs_soc; //Fo = Fphy+ Fsoc
	printf("object rep_static init = %.3f\n", object1.rep_static);

	if(object1.distance > 40){
		object1.rep_static = 0;
	}

	if(object1.distance <= 200){
		object_detect = true;
	}

	if(object1.distance <= 200){
		object_rel_dist.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/object_rel_dis.txt", ios::in | ios::out | ios::app);
		object_rel_dist << object1.distance/scaleFactor << endl;
		object_rel_dist.close();
	}
}


void SF_goal_1(float xTarget, float yTarget, float tTarget){
	const float T_S = 0.05; //0.05
	float target_radius = 5;
	float d_velo = 0.8;		//desired velocity
	float r_velo; 			//velocity saat ini
	float robot_distance;
	float robot_disx, robot_disy;
	float m,beta,teta;
	

	target.desired_speed = 40;
	m=1;
	//printf("loop = %d\n",loop);
	target.posx = xTarget;
	target.posy = yTarget;
	target.post = tTarget;
	printf("xTarget = %f, yTarget = %f, tTarget = %f\n",target.posx,target.posy,target.post);

	printf("robot.odom_x = %.3f, y = %.3f\n",robot.odom_x,robot.odom_y);
	robot.new_posx = centerX + (robot.odom_x * scaleFactor);
	robot.new_posy = centerY - (robot.odom_y * scaleFactor);

	printf("robot.new_posx = %.3f, y = %.3f\n",robot.new_posx,robot.new_posy);
	target.distanceX = target.posx - robot.new_posx;
	target.distanceY = - target.posy + robot.new_posy;
	printf("target.distanceX = %.3f, y = %.3f\n",target.distanceX,target.distanceY);
	target.distance = (float)(sqrt((double)target.distanceX*(double)target.distanceX + (double)target.distanceY*(double)target.distanceY)); 
	target.theta = 	(float) abs(getDegree(target.distanceX,target.distanceY));
	beta = (atan(target.distanceY/target.distanceX)*180/pi);
	//teta = atan(robot.odom_y/robot.odom_x)*180/pi;
	printf("robot.gyro_z = %.3f, robot.new_gyro_z = %.3f\n",robot.gyro_z,robot.new_gyro_z);
	if(robot.gyro_z>180)
	{
		robotgyro_z = 360-robot.gyro_z;
	}
	else
	{
		robotgyro_z = -1*robot.gyro_z;
	}
	printf("robotgyro_z = %.3f\n",robotgyro_z);
	printf("robot.odom_t = %.3f\n",robot.odom_t);
	printf("beta = %.3f\n",beta);
	printf("target.distance = %.3f\t", target.distance);
	printf("target.theta = %.3f\n", target.theta);

	//target.forcealpha = ((target.desired_speed * target.distance) - target.distance ) * (1/T_S);
	
	target.forcealpha = m*(target.desired_speed - robot.line_velo) / T_S; //?? Gak ada mnya?
	printf("diff velocity = %.3f\t",(target.desired_speed - robot.line_velo));
	printf("robot.line_velo = %f\n",robot.line_velo);
	printf("target forcealpha = %.3f\n", target.forcealpha);

	robot_vel.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/vt_robot.txt", ios::in | ios::out | ios::app);
	robot_vel << robot.line_velo/scaleFactor << endl;
	robot_vel.close();
	target.kecepatan = target.forcealpha;
	/*target.error = target.forcealpha;
		//2, 0.25, 0.01 bisa!!!!!
	//==========PID TARGET============// //2, 0.25, 0.01 bisa	//2, 0.5, 0.01 bisa
	target.proportional = 0.5* target.error; //0.105 goyang //0.3 //1 nabrak person	//3 gk bs
	target.derivative   = 0.25*(target.error - target.lasterror)/T_S; //0.0025 //0.2 //0.8  //1.025
	target.integral     = 0.01*target.sumError*T_S; //0.0001 //0.1 //0.7 //1.0001
	target.lasterror    = target.error;
	target.sumError    += target.error;
	//printf("target.sumError = %.3f\t",target.sumError);
	
	if(target.sumError>4000){
		target.sumError=4000;}	//max force??
	else if(target.sumError<-4000){
		target.sumError=-4000;}	//min force??
	target.kecepatan=target.proportional+target.derivative+target.integral;*/
	//=================END==================// 
	//target.speed = 0;
	//printf("target.sumError = %.3f\n",target.sumError);
	//printf("trgt spd = %.3f\n", target.speed);
	printf("trgt spd = %.3f\n", target.kecepatan);

	//==========PID Heading============//
	//Input 0<>179 ~ -1<>-180///CW Imu negative
	//need to look the result from input

	target.gaya_navigasi = target.kecepatan + object1.rep_static; //Fnavigasi
	printf("target.gaya_navigasi = %.3f\n",target.gaya_navigasi);

	//target.errorTheta        = beta - (robotgyro_z);	
	target.errorTheta        = (robotgyro_z);	
	//printf("robot.post = %.3f\t", robot.post);
	printf("target.errorTheta = %.3f\n", target.errorTheta);
	if(target.errorTheta >180){
		target.errorTheta -= 360;}
	else if(target.errorTheta <-180){
		target.errorTheta += 360;}
	printf("errortheta normal = %f \n ",target.errorTheta );
	target.speedTheta = target.errorTheta;
	

	n_sdt_scan++; //??
	total_sdt_err = total_sdt_err + abs(target.errorTheta);

	printf("n_sdt_scan = %d \n ",n_sdt_scan );
	printf("total_sdt_err = %.3f \n ",total_sdt_err );
/*
	target.sumErrorTheta = target.sumErrorTheta+target.errorTheta;
	printf("target.sumErrorTheta = %.3f\n",target.sumErrorTheta);

	target.proportionalTheta = 1.86*target.errorTheta;
	target.derivativeTheta   = 0.008*(target.errorTheta-target.lasterrorTheta)/T_S;
	target.integralTheta	 = 0.01*target.sumErrorTheta;
	target.lasterrorTheta    = target.errorTheta;
	target.speedTheta 	 = target.proportionalTheta+target.derivativeTheta+target.integralTheta;
	
	if(target.sumErrorTheta>1000){target.sumErrorTheta=1000;}
	else if(target.sumErrorTheta<-1000){target.sumErrorTheta=-1000;}
	*/
	//target.speedTheta = 1*target.speedTheta;

	printf("target.speedTheta = %.3f\n", target.speedTheta);
	
	target.speed = target.gaya_navigasi*T_S;
	printf("target.speed = %.3f\n",target.speed);
	alpa = atan((target.forcealpha*sin(beta/57.2957795)+object1.rep_static*sin(sudut_object1/57.2957795))/(target.forcealpha*cos(beta/57.2957795)+object1.rep_static*cos(sudut_object1/57.2957795)))*180/pi;
	printf("alpa = %.3f\n",alpa);

	target.speedx = target.speed * cos(alpa/57.2957795);
	target.speedy = target.speed * sin(alpa/57.2957795);
	target.speedt = target.speedTheta;
	//target.speedt = 0;

	/*target.speedx = 19.61;
	target.speedy = 3.922;
	target.speedt = 11.31;	*/

	printf("trgt speedx = %.3f\n", target.speedx);
	printf("trgt speedy = %.3f\n", target.speedy);
	printf("trgt speedt = %.3f\n", target.speedt);
	

	//target.speedx = target.speedx + object1.speedx + person.speedx + person.speedxx;
	//target.speedy = target.speedy + object1.speedy + person.speedy + person.speedyy;
	//target.speedx = target.speedx;
	//target.speedy = target.speedy;
	//target.speedt = target.speedt;
	//printf("robot speedx = %.3f\n", target.speedx);
	//printf("robot speedy = %.3f\n", target.speedy);
	

	if(target.distance > target_radius){
		robotgerak_c(target.speedx ,target.speedy,target.speedt);
		target_detect = true;
		}
	else{ 
		if(target.goal_counts <2){
			target.goal_counts++;
		}
		robotgerak_c(0,0,0); 
	
	} 
	 

}


void linearVelocity(){
	float dist_x, dist_y;
	float distance;
	float velocity;
	float T_s = 0.05; //0.05

	dist_x = (robot.line_velo_x * T_s) * scaleFactor;
	dist_y = (robot.line_velo_y * T_s) * scaleFactor;

	distance = (float)(sqrt((double)dist_x*(double)dist_x + (double)dist_y*(double)dist_y)); 
	velocity = distance / T_s;
	robot.line_velo = velocity;
	printf("robot.line_velo = %.3f\n", velocity);
}






//////////////////////////////////// ROBOT SUBSCRIBER ///////////////////////////
void posXcallback(const std_msgs::Float32::ConstPtr& msg)
{
	robot.posx = msg->data;					//VREP (m)
	
 // ROS_INFO("I heard: [%f]", robot.posx);
}
void posYcallback(const std_msgs::Float32::ConstPtr& msg)
{	
	robot.posy =  msg->data;				//VREP (m)
	
 //ROS_INFO("I heard: [%f]", robot.posy);
}
void posTcallback(const std_msgs::Float32::ConstPtr& msg)
{ 
	robot.post =  msg->data;
	robot.new_post = robot.post * 0.01745329252;	//(robot.post * PI/180)
	//t_cor = robot.new_post;
	//t_pos = robot.new_post;
	//printf("post = %f\n", robot.post);

 // ROS_INFO("I heard: [%f]", robot.post);
}

/////////////////PERSON 1 POSISI SUBSCRIBER///////////////
void posXXcallback(const std_msgs::Float32::ConstPtr& msg)
{
	person.posxx = msg->data;					//VREP (m)

 // ROS_INFO("I heard: [%f]", robot.posx);
}
void posYYcallback(const std_msgs::Float32::ConstPtr& msg)
{
	person.posyy =  msg->data;				//VREP (m)

 //ROS_INFO("I heard: [%f]", robot.posy);
}
void odoXXcallback(const std_msgs::Float32::ConstPtr& msg)
{
	person.odomxx = msg->data;
	//robot.l_odomx = robot.odom_x;
	//printf("robot.odom_x = %f\n", robot.odom_x);
}
void odoYYcallback(const std_msgs::Float32::ConstPtr& msg)
{
	person.odomyy =  msg->data;
	//robot.l_odomy = robot.odom_y;
	//printf("robot.odom_y = %f\n", robot.odom_y);
}

/////////////////PERSON 2 POSISI SUBSCRIBER///////////////
void possXXcallback(const std_msgs::Float32::ConstPtr& msg)
{
	person.possxx = msg->data;					//VREP (m)

 // ROS_INFO("I heard: [%f]", robot.posx);
}
void possYYcallback(const std_msgs::Float32::ConstPtr& msg)
{
	person.possyy =  msg->data;				//VREP (m)

 //ROS_INFO("I heard: [%f]", robot.posy);
}
void odooXXcallback(const std_msgs::Float32::ConstPtr& msg)
{
	person.odommxx = msg->data;
	//robot.l_odomx = robot.odom_x;
	//printf("robot.odom_x = %f\n", robot.odom_x);
}
void odooYYcallback(const std_msgs::Float32::ConstPtr& msg)
{
	person.odommyy =  msg->data;
	//robot.l_odomy = robot.odom_y;
	//printf("robot.odom_y = %f\n", robot.odom_y);
}

void odoXcallback(const std_msgs::Float32::ConstPtr& msg)
{
	robot.odom_x = msg->data;
	robot.l_odomx = robot.odom_x;
	//printf("robot.odom_x = %f\n", robot.odom_x);

}
void odoYcallback(const std_msgs::Float32::ConstPtr& msg)
{	
	robot.odom_y =  msg->data;  
	robot.l_odomy = robot.odom_y;
	//printf("robot.odom_y = %f\n", robot.odom_y);
	
}
void odoTcallback(const std_msgs::Float32::ConstPtr& msg)
{ 
	robot.odom_t =  msg->data;
	robot.odom_t = robot.odom_t * pi / 180;
	//robot.l_odomt = robot.odom_t;
}


void gyroXCallback(const std_msgs::Float32::ConstPtr& msg){
	robot.gyro_x = msg->data;
}
void gyroYCallback(const std_msgs::Float32::ConstPtr& msg){
	robot.gyro_y = msg->data;
}
void gyroZCallback(const std_msgs::Float32::ConstPtr& msg){
	robot.gyro_z = msg->data;
	robot.new_gyro_z = (robot.gyro_z-90) * pi / 180; // (robot.gyro_z + degree)*(pi/180) [perhatikan coordinat vrep]
	robot.l_gyroz = robot.new_gyro_z;
	//printf("gyro_z = %f\n", robot.gyro_z);
}



/////////////////// OBJECT SUBSCRIBER /////////////////////////
void obj_posX_callback(const std_msgs::Float32::ConstPtr& msg)
{ 
	object1.pos_x =  msg->data;					//VREP
	
	object1.new_pos_x = centerX + (object1.pos_x - robot.posx + robot.odom_x)*scaleFactor;
 // ROS_INFO("I heard: [%f]", robot.post);
}
void obj_posY_callback(const std_msgs::Float32::ConstPtr& msg)
{ 
	object1.pos_y =  msg->data;					//VREP
	object1.new_pos_y = centerY - (object1.pos_y - robot.posy + robot.odom_y)*scaleFactor;
 // ROS_INFO("I heard: [%f]", robot.post);
}
void obj_posZ_callback(const std_msgs::Float32::ConstPtr& msg)
{ 
	object1.pos_z=  msg->data;					//VREP
 // ROS_INFO("I heard: [%f]", robot.post);
}
void obj_posT_callback(const std_msgs::Float32::ConstPtr& msg)
{ 
	object1.pos_t =  msg->data;					//VREP
 // ROS_INFO("I heard: [%f]", robot.post);
}


void velo_linear_x_callback(const std_msgs::Float32::ConstPtr& msg){
	robot.line_velo_x = msg->data;

}

void velo_linear_y_callback(const std_msgs::Float32::ConstPtr& msg){
	robot.line_velo_y = msg->data;

}




void personfCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
	float posx, posy;
	person.posx = 0; person.posy = 0 ;
	person.posx = msg->data[0];
	person.posy = msg->data[1];

	printf("X = %.3f, Y = %.3f\n", person.posx, person.posy);

	person_detect = true;
}

void person_msgCallback(const tes_vrep::Person::ConstPtr& msg){
	uint8_t another;
	another = msg->another;
	float dist_t;
	float min_dist = 10000;
	int arr_min;
	//float size_p;

	float dr_x, dr_y;

	dr_x = centerX + (robot.odom_x * scaleFactor);
	dr_y = centerY - (robot.odom_y * scaleFactor);

	//size_p = msg-> person.size();

	if (another != 0){
		for(int i = 0; i < msg->another; i++){
			dist_t = getDistance(dr_x, dr_y, msg->person[i].x, msg->person[i].y);
			min_dist = MIN(min_dist, dist_t);
			if(min_dist == dist_t){
				arr_min = i;
			}
			//printf("dist_t = %.4f\n", dist_t);
			
		}
		//printf("min_dist_t = %.4f\n", min_dist);
		person.posx_msg = msg->person[arr_min].x;
		person.posy_msg = msg->person[arr_min].y;
		//printf("X_msg = %.4f, Y_msg = %.4f\n", msg->person[arr_min].x, msg->person[arr_min].y);
	}

	//printf("another msg = %d\n", another);

	//person_detect = true;
}



////////////////////////////// LIDAR SUBSCRIBER /////////////////////////////
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    double angle, distance, degree;
	float tmp = 9999;
    int count = scan->scan_time / scan->time_increment;
    scan_size = scan->ranges.size();

	for(size_t i = 0; i < scan->ranges.size(); i++){
        angle = RAD2DEG(scan->angle_min + i*scan->angle_increment);
        degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        laserData[i] = scan->ranges[i];
	
		//printf("%0.3f    ", laserData[i]);
		if(laserData[i] > 0){
			if(tmp > laserData[i]) {

				tmp=laserData[i];
				min_dist_lid = laserData[i];
				min_dist_lid_n = i;
			}
		}
	}

    
    scanready = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigasi");

	ros::NodeHandle n;
	float timer=0.01;//10 ms
    ros::Timer timer1 = n.createTimer(ros::Duration(timer), timeSamplingCallback);

	ros::Subscriber posx = n.subscribe<std_msgs::Float32>("/posx",200,posXcallback);
	ros::Subscriber posy = n.subscribe<std_msgs::Float32>("/posy",200,posYcallback);
	ros::Subscriber post = n.subscribe<std_msgs::Float32>("/post",200,posTcallback);

	//KOORDINAT PERSON 1
	ros::Subscriber posxx = n.subscribe<std_msgs::Float32>("/posxx",200,posXXcallback);
	ros::Subscriber posyy = n.subscribe<std_msgs::Float32>("/posyy",200,posYYcallback);
	//ODOM PERSON
	ros::Subscriber odomxx = n.subscribe<std_msgs::Float32>("/odom_xx",200,odoXXcallback);
	ros::Subscriber odomyy = n.subscribe<std_msgs::Float32>("/odom_yy",200,odoYYcallback);

	//KOORDINAT PERSON 2
	ros::Subscriber possxx = n.subscribe<std_msgs::Float32>("/possxx",200,possXXcallback);
	ros::Subscriber possyy = n.subscribe<std_msgs::Float32>("/possyy",200,possYYcallback);
	//ODOM PERSON
	ros::Subscriber odommxx = n.subscribe<std_msgs::Float32>("/odomm_xx",200,odooXXcallback);
	ros::Subscriber odommyy = n.subscribe<std_msgs::Float32>("/odomm_yy",200,odooYYcallback);

	ros::Subscriber odomx = n.subscribe<std_msgs::Float32>("/odom_x",200,odoXcallback);
	ros::Subscriber odomy = n.subscribe<std_msgs::Float32>("/odom_y",200,odoYcallback);
	ros::Subscriber odomt = n.subscribe<std_msgs::Float32>("/odom_t",200,odoTcallback);
    
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/lasserDat", 2000, laserCallback);
	
	//ros::Subscriber gyro_x_sub = n.subscribe<std_msgs::Float32>("/gyro_x_dat", 100, gyroXCallback);
	//ros::Subscriber gyro_y_sub = n.subscribe<std_msgs::Float32>("/gyro_y_dat", 100, gyroYCallback);
	ros::Subscriber gyro_z_sub = n.subscribe<std_msgs::Float32>("/gyro_z_dat", 100, gyroZCallback);

	ros::Subscriber person_msg_sub = n.subscribe<tes_vrep::Person>("/person_msg",200,person_msgCallback);
	ros::Subscriber personf_sub = n.subscribe<std_msgs::Float64MultiArray>("/person_f",200,personfCallback);
	
	motor1pub = n.advertise<std_msgs::Float32> ("/vel/motor1",500);
	motor2pub = n.advertise<std_msgs::Float32> ("/vel/motor2",500);
	motor3pub = n.advertise<std_msgs::Float32> ("/vel/motor3",500);
	motor4pub = n.advertise<std_msgs::Float32> ("/vel/motor4",500);

	obj1_pos_x = n.subscribe<std_msgs::Float32>("/obs1/posx", 100, obj_posX_callback);
	obj1_pos_y = n.subscribe<std_msgs::Float32>("/obs1/posy", 100, obj_posY_callback);
	obj1_pos_z = n.subscribe<std_msgs::Float32>("/obs1/posz", 100, obj_posZ_callback);
	obj1_pos_t = n.subscribe<std_msgs::Float32>("/obs1/post", 100, obj_posT_callback);


	ros::Subscriber velo_linear_x_sub = n.subscribe<std_msgs::Float32>("/lin_velo_x",200,velo_linear_x_callback);
	ros::Subscriber velo_linear_y_sub = n.subscribe<std_msgs::Float32>("/lin_velo_y",200,velo_linear_y_callback);


	
	ros::Duration rate_0(20);
    rate_0.sleep();
	ros::Rate rate_1(100);

	

	for(int i=0;i<imgArr;i++){
	    imgSave[i]=Mat(800, 800, CV_8UC3, Scalar(0));
	}

	float target_x, target_y, target_t;
	float targetxx, targetyy, targettt;
	
	std::chrono::duration<double> elapsed_seconds;
	while(ros::ok()){

		if(scanready){
			//auto start_tt = std::chrono::system_clock::now();
			//cout << "elapsed time" << elapsed_seconds.count() << "s\n";

			//target_x = centerX + (5 - robot.posx + robot.odom_x) * scaleFactor;
			//target_y = centerY - (1.5 - robot.posy + robot.odom_y) * scaleFactor;
			float tg_x, tg_y;
			float ob_x, ob_y;
			float per_x, per_y;

			target_x = centerX + (goal[target.goal_counts][0] - robot.posx + robot.odom_x) * scaleFactor;
			target_y = centerY - (goal[target.goal_counts][1] - robot.posy + robot.odom_y) * scaleFactor;
			target_t = goal[target.goal_counts][2];
			/*
			targetxx = target_x - robot.odom_x;
			targetyy = target_y - robot.odom_y;
			targettt = target_t - robot.odom_t;
			*/
			/*
			printf("person.odomxx = %.3f\t", person.odomxx);
			printf("person.odomyy = %.3f\t", person.odomyy);
			*/
			targetxx = centerX + (robot.posx + robot.odom_x) * scaleFactor;
			targetyy = centerY - (robot.posy + robot.odom_y) * scaleFactor;
			printf("targetxx = %f, targetyy = %f\n", targetxx, targetyy);

			//printf("robot.posxx = %.3f\t", robot.posxx);
			//printf("robot.posyy = %.3f\n", robot.posyy);


			/*
			printf("robot.odom_x = %.3f\t", robot.odom_x);
			printf("robot.odom_y = %.3f\n", robot.odom_y);
			*/
			/*
			printf("target_x = %.3f, ", target_x);
			printf("target_y = %.3f, ", target_y);
			printf("target_y = %.3f\n", target_t);
			*/
			/*
			printf("person.posx_msg = %.3f\t", person.posx_msg);
			printf("person.posy_msg = %.3f\n", person.posy_msg);
			*/
			//printf("person.possxx = %.3f\t", person.possxx);
			//printf("person.possyy = %.3f\n", person.possyy);
			//printf("person.odommxx = %.3f\t", person.odommxx);
			//printf("person.odommyy = %.3f\n", person.odommyy);

			//printf("person.posxx = %.3f\t", person.posxx);
			//printf("person.posyy = %.3f\n", person.posyy);
			/*
			float jarak,jarakx,jaraky;
			jarakx = (robot.posx - person.posxx) *scaleFactor;
			jaraky = (robot.posy - person.posyy) *scaleFactor;
			jarak = sqrt(jarakx*jarakx + jaraky*jaraky);
			printf("jarak = %.3f\n",jarak);
			double xx,ret, val;
			float sudutt;
					xx = jarakx/jarak;
					xx=abs(xx);
					val = 180.0 / pi;
					sudutt = acos(xx) * val;
					if(person.dist_x<0) //person sudah lewat.
					{
						sudutt=180-sudutt;
					}
					printf("sudut = %.3f\n",sudutt);
			*/
			//printf("robot.line_velo_x = %.3f\t", robot.line_velo_x);
			//printf("robot.line_velo_y = %.3f\n", robot.line_velo_y);

			linearVelocity();

			//SF_static_obj_3();
			//fuzzySFM();
			//SF_dyn();
			//SF_person_1();
			//SF_person_2();
			SF_goal_1(target_x, target_y, target_t);
			//robotgerak_c(-500, 300 , 0);
			
			
			float angle_step;
			angle_step = (float)270 / (float)scan_size;
			img = 0;
			
			for(int i = 0; i <= scan_size; i++){
				degx = cos(((angle_step*(i))-45) * M_PI / 180); //rad
				degy = sin(((angle_step*(i))-45) * M_PI / 180);

				dx = (laserData[i] * scaleFactor)*degx;	//rad * distance	//(0.5*distance) * cos(((angle_step*(i))-45) * M_PI / 180)
				dy = (laserData[i] * scaleFactor)*degy;

				x_coo = (cos(robot.new_gyro_z) * (dx - 0)) - (sin(robot.new_gyro_z) * (dy - 0)) + (centerX + (robot.odom_x * scaleFactor));
				y_coo = (sin(robot.new_gyro_z) * (dx - 0)) + (cos(robot.new_gyro_z) * (dy - 0)) - (centerY - (robot.odom_y * scaleFactor));


				y_coo = -y_coo;
				
				
				cv::circle(img, Point(x_coo,y_coo),1, Scalar(255,0,0),1);
				
			}


			dr_x = centerX + (robot.odom_x * scaleFactor);
			dr_y = centerY - (robot.odom_y * scaleFactor);
			cv::circle(img, Point(dr_x, dr_y), 1, Scalar(0,250,0), -1);

			//cv::circle(img, Point(object1.new_pos_x, object1.new_pos_y), 1, Scalar(150,250,0), -1);

			cv::circle(img, Point(target_x, target_y), 1, Scalar(0,150,250), -1);

			cv::circle(img, Point(object1.m_pos_x, object1.m_pos_y), 5, Scalar(255,0,127), -1);

			//printf("person_detect = %d\n", person_detect);

			if(target_detect){
				tg_x = (35 ) * sin( target.theta * M_PI / 180) + dr_x;
				tg_y = (35 ) * cos( target.theta * M_PI / 180) - dr_y;
				tg_y = -tg_y;
				
				arrowedLine(img, Point(dr_x, dr_y), Point(tg_x, tg_y), Scalar(0, 225, 0), 0.7);

				target_detect = false;
			}

			if(object_detect){
				ob_x = (25 ) * sin( (object1.theta+180) * M_PI / 180) + dr_x;
				ob_y = (25 ) * cos( (object1.theta+180) * M_PI / 180) - dr_y;
				ob_y = -ob_y;

				arrowedLine(img, Point(dr_x, dr_y), Point(ob_x, ob_y), Scalar(255,0,127), 0.5);
			
				object_detect = false;
			}

			/*
			if(person_detect){

				Rect recta(person.posx_msg, person.posy_msg, 25, 17);
				rectangle(img, recta,  cv::Scalar(200, 210, 140));



				//person_detect = false;
			}

			if(person_detect1){
				per_x = (25 ) * sin( (person.theta+180) * M_PI / 180) + dr_x;
				per_y = (25 ) * cos( (person.theta+180) * M_PI / 180) - dr_y;
				per_y = -per_y;

				arrowedLine(img, Point(dr_x, dr_y), Point(per_x, per_y), Scalar(200, 210, 140), 0.5);
				person_detect1 = false;
			}
*/

			
			cv::namedWindow("Display", cv::WINDOW_AUTOSIZE);
			cv::imshow("Display", img);
			char s = cv::waitKey(1);

			//auto end_tt = std::chrono::system_clock::now();
			//elapsed_seconds = end_tt - start_tt;

		}



		ros::spinOnce();
        rate_1.sleep();
	}
	return 0;
}
