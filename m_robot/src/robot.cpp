#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <string>     // to use std::string, std::to_string() and "+" operator acting on strings

#define PORT    8080
#define MAXLINE 1024

double leftWheel = 0;
double rightWheel = 0;

double leftOdom = 0;
double rightOdom = 0;

void subscribeLeft(const std_msgs::Float64 &left)
{
	leftWheel = left.data;
}

void subscribeRight(const std_msgs::Float64 &right)
{
	rightWheel = right.data;
}


/*TIP: pridat moznost se pripojit na ruzne ip adresy*/
int main(int argc, char **argv)
{
	// Init
	ros::init(argc, argv, "robot");
	ros::NodeHandle nh;

	// Subscriber
	ros::Subscriber s_left = nh.subscribe("s_leftWheel", 1, subscribeLeft);
	ros::Subscriber	s_right = nh.subscribe("s_rightWheel", 1, subscribeRight);


	char buffer[1000];
	int sockfd, n;
	struct sockaddr_in servaddr;

	// clear servaddr
	bzero(&servaddr, sizeof(servaddr));
	servaddr.sin_addr.s_addr = inet_addr("192.168.1.101");
	servaddr.sin_port = htons(PORT);
	servaddr.sin_family = AF_INET;

	// create datagram socket
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);

	// connect to server
	if(connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
		printf("\n Error : Connect Failed \n");
		exit(0);
	}

	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		std::cout << "1" << std::endl;
		std::ostringstream spd;
		spd << "spd," << leftWheel << "," << rightWheel;
		const std::string s_spd = spd.str();
		const char* c_spd = s_spd.c_str();
		sendto(sockfd, c_spd, MAXLINE, 0, (struct sockaddr*)NULL, sizeof(servaddr));
		recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)NULL, NULL); //v bufferu je odpoved ze serveru -robota

		std::cout << "2" << std::endl;
		std::ostringstream odm;
		const std::string s_odm = odm.str();
		const char* c_odm = s_odm.c_str();
		odm << "odm";
		sendto(sockfd, c_odm, MAXLINE, 0, (struct sockaddr*)NULL, sizeof(servaddr));
		recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)NULL, NULL); //v bufferu je odpoved ze serveru -robota

		std::cout <<  "2" << std::endl;
		// poslat data z enkoderu ...
		ros::spinOnce();
		loop_rate.sleep();
	}
	close(sockfd);
	return 0;
}
