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

class Controller{
    public:
	int fd = 0;		//com port number
	char buff[50];
	char port_name[20];
	int baud_rate;
	int iread;
	float x_offset_param = -0.1f;
	float y_offset_param = 0.1f;
	float th_offset_param = -0.1f;
	float time_offset_param = 1.0f;		//unused now
	float x = 0.0f;
	float y = 0.0f;
	float th = 0.0f;
	float vx = 0.1f;
	float vy = -0.1f;
	float vth = 0.0f;
	float dt;
	float delta_x;
	float delta_y;
	float delta_th;
	char sendbuff[100] = "z 20 20\r\n";
	int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
	int open_port(int fd,int comport,char* port_name);
	void chatterCallback(const geometry_msgs::Twist& vel);
	void connectPort(char* port_name,int baud_rate);
	void spinCallback(const ros::TimerEvent&);
	explicit Controller(ros::NodeHandle& n);
	~Controller();

    protected:
	ros::Publisher  cmd_pub;
	ros::Subscriber sub;

	nav_msgs::Odometry odom;
	geometry_msgs::Quaternion odom_quat;
	geometry_msgs::TransformStamped odom_trans;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Time current_time = ros::Time::now();
	ros::Time last_time = ros::Time::now();
        std_msgs::String msg;

};

Controller::Controller(ros::NodeHandle& n){
	std::string s;
	n.param<std::string>("port_name", s, "/dev/ttyACM0");		//only string type needs defination
	n.param("baud_rate", baud_rate, 115200);
	strcpy(port_name, s.c_str());	
	try{
		connectPort(port_name,baud_rate);
	}	
	catch (...) {std::cout<<"connection failed!";}
	sub = n.subscribe("cmd_vel", 2, &Controller::chatterCallback, this);
        //cmd_pub = n.advertise<nav_msgs::Odometry>("cmd", 2);
	write(fd, sendbuff, strlen(sendbuff));
}
	
Controller::~Controller(){
	close(fd);
}

//---------------member functions----------------//

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
    if(set_opt(fd,baud_rate,8,'N',1)<0)
    {
        perror("set_opt error");
        return;
    }
    printf("fd=%d\n",fd);
}

//subscirbe the command and write to driver
void Controller::chatterCallback(const geometry_msgs::Twist& vel)
{	
	int vl, vr;	
	vl = 106 * vel.linear.x + vel.angular.z * 19;
	vr = 106 * vel.linear.x - vel.angular.z * 19; 
	std::stringstream ss;
	//ss << "hello world again" << count;
	ss << "z "<< vl << " " << vr << ";";	
	strcpy(sendbuff, ss.str().c_str());
	int temp = strlen(sendbuff);
	sendbuff[temp] = '\r';
	sendbuff[temp+1] = '\n';
	//std::cout << vl << " and " << vr << std::endl;
	std::cout << "buffer is "<< sendbuff   << std::endl;
	write(fd, sendbuff, strlen(sendbuff));	//write to st
	memset(sendbuff, 0, 100 );
}


//---------------declaration ends----------------//

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	Controller C(n);	
	ros::spin();			//subscribe info is triggerred in callback
	//loop_rate.sleep();			
	ROS_INFO("WTF!!");
	//close(C.fd);				//comment it or not?
	return 0;
}

