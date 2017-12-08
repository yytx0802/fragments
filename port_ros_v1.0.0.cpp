#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <unistd.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

char sendbuff[100] = "0,0,0\r\n";

class Controller{
    public:
	int fd = 0;
	int i; 
	std::string s;
	char port_name[20];
	int baud_rate;


	int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
	int open_port(int fd,int comport,char* port_name);
	void chatterCallback(const geometry_msgs::Twist& vel);
	void connectPort(char* port_name,int baud_rate);
	explicit Controller(ros::NodeHandle& n);
	~Controller();
    protected:
	ros::NodeHandle m_node;
	ros::Publisher  m_joint_pub;
	ros::Subscriber m_joint_sub;

};
Controller::Controller(ros::NodeHandle& n){
	n.param<std::string>("port_name", s, "/dev/ttyACM0");
	n.param("baud_rate", baud_rate, 115200);
	strcpy(port_name, s.c_str());
	connectPort(port_name,baud_rate);
}	
Controller::~Controller(){
	close(fd);
}
int Controller::set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }

    switch( nEvent )
    {
    case 'O':                     //奇校验
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E':                     //偶校验
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':                    //无校验
        newtio.c_cflag &= ~PARENB;
        break;
    }

switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    if( nStop == 1 )
    {
        newtio.c_cflag &=  ~CSTOPB;
    }
    else if ( nStop == 2 )
    {
        newtio.c_cflag |=  CSTOPB;
    }
    newtio.c_oflag = 0;
    newtio.c_iflag = IGNPAR | ICRNL;
    newtio.c_lflag = ICANON;
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

int Controller::open_port(int fd,int comport,char* port_name)
{
    //char *dev[]={"/dev/ttyACM0","/dev/ttyUSB0","/dev/ttyS2"};
    if (comport==1)
    {    fd = open( port_name, O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyACM .....\n");
        }
    }
    else if(comport==2)
    {    fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyS1 .....\n");
        }
    }
    else if (comport==3)
    {
        fd = open( "/dev/ttyS2", O_RDWR|O_NOCTTY|O_NDELAY);
        if (-1 == fd)
        {
            perror("Can't Open Serial Port");
            return(-1);
        }
        else
        {
            printf("open ttyS2 .....\n");
        }
    }
    if(fcntl(fd, F_SETFL, FNDELAY)<0)
    {
        printf("fcntl failed!\n");
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,FNDELAY));
    }
    if(isatty(STDIN_FILENO)==0)
    {
        printf("standard input is not a terminal device\n");
    }
    else
    {
        printf("isatty success!\n");
    }
    printf("fd-open=%d\n",fd);
    return fd;
}

void Controller::connectPort(char* port_name,int baud_rate){
    if((fd=open_port(fd,1,port_name))<0)
    {
        perror("open_port error");
        return;
    }
    if((i=set_opt(fd,baud_rate,8,'N',1))<0)
    {
        perror("set_opt error");
        return;
    }
    printf("fd=%d\n",fd);
}

void Controller::chatterCallback(const geometry_msgs::Twist& vel)
{

	std::stringstream ss;
	//ss << "hello world again" << count;
	ss <<vel.linear.x<<","<<vel.linear.y<<","<<vel.angular.z;
	strcpy(sendbuff, ss.str().c_str());
	int temp = strlen(sendbuff);
	sendbuff[temp] = '\r';
	sendbuff[temp+1] = '\n';
	//std::cout << "buffer is "<< sendbuff << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
        int nread,nwrite;
        char  buff[100];
	char port_name[20];
	int baud_rate;
        std_msgs::String msg;
	std::string s;

	float x = 0.0f;
	float y = 0.0f;
	float th = 0.0f;
	float vx = 0.1f;
	float vy = -0.1f;
	float vth = 0.1f;
	float dt;
	float delta_x;
	float delta_y;
	float delta_th;
	ros::Time current_time, last_time;
        ros::NodeHandle n;

	nav_msgs::Odometry odom;
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	geometry_msgs::Quaternion odom_quat;
	geometry_msgs::TransformStamped odom_trans;
	tf::TransformBroadcaster odom_broadcaster;
	strcpy(port_name, s.c_str());
        ros::Rate loop_rate(10);
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	Controller C(n);
        ros::Subscriber sub = n.subscribe("cmd_vel", 10, &Controller::chatterCallback,&C);
        ros::Publisher cmd_pub = n.advertise<nav_msgs::Odometry>("command", 10);
	while (ros::ok()) //循环读取数据
	{
	    if((nread = read(C.fd, buff, 100))>2)		//read from st
	    {
		//printf( "%s", buff);
		current_time = ros::Time::now();
		sscanf(buff, "%f, %f, %f", &vx, &vy, &vth);					//x, y,th
		dt = (current_time - last_time).toSec();			//still needs consideration 0.492032
		last_time = ros::Time::now();
		delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		delta_th = vth * dt;
		printf("dt is %f, x is %f\n", dt, x);
		x += delta_x;
		y += delta_y;
		th += delta_th;
		odom_quat = tf::createQuaternionMsgFromYaw(th);
		odom_trans.header.stamp = current_time;

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		odom.header.stamp = current_time;

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

	        std::string str(buff);
	        msg.data = str;
	        cmd_pub.publish(odom);				//publish info to the "command" topic
		nwrite = write(C.fd, sendbuff, strlen(sendbuff));	//write to st
		memset( buff, 0, 100 );
		ros::spinOnce();					//subscribe info is triggerred in callback
		loop_rate.sleep();		//0.1s  CPU 100%
	    }
	}
	ROS_INFO("WTF!!");
	//close(C.fd);				//comment it or not?
	return 0;
}
