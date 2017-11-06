#include <ros/ros.h>
#include <arithmetic_node/arithmetic_reply.h>
#include <message_ui/sent_msg.h>
#include <sstream>
#include <string>
#include <iostream>

using namespace std;
ros::Publisher arith_pub;
ros::Subscriber arith_sub;
float num1;
float num2;
string oper;
string msg_str;
string num_len;
ros::Time time_received;
ros::Time time_answered;
ros::Duration process_time;
int dont_cont = 0;

void arith_callback(const message_ui::sent_msg::ConstPtr& msg){
	arithmetic_node::arithmetic_reply math;
	time_received = ros::Time::now();
	//ROS_INFO_STREAM(time_received.toSec());
	stringstream aa;
	
	if (int(msg->message[0]) > 47 && int(msg->message[0]) < 58){
			aa << msg->message << ' ';
			msg_str = aa.str();
			int count = 0;
			int pose = 0;
			//ROS_INFO_STREAM(msg_str);
			dont_cont = 0;
			//ROS_INFO_STREAM(dont_cont);
			for(char & c : msg_str){
				
				if (c == '+' || c == '-' || c == '*' || c == '/'){
					pose = count;
					//ROS_INFO_STREAM(pose);
				}
				
				count++;
			}
			//ROS_INFO_STREAM(dont_cont);
			
			//ROS_INFO_STREAM(msg_str.substr(0,pose));
			//ROS_INFO_STREAM(msg_str.substr(pose+1,(msg_str.length()-pose)));
			string n1_str = msg_str.substr(0,pose);
			string n2_str = msg_str.substr(pose+1,(msg_str.length()-(pose+1)));
			for(char & c : n1_str){
				//ROS_INFO_STREAM(int(c));
				if ((int(c) > 46 || int(c) < 57) || int(c) == 32){
					
					//ROS_INFO_STREAM(dont_cont);
				} 
				else{
					dont_cont = 1;
				}
			}
			for(char & c : n2_str){
				//ROS_INFO_STREAM(int(c));
				if ((int(c) > 46 && int(c) < 57) || int(c) == 32) {
					
					//ROS_INFO_STREAM(dont_cont);
				} 
				else{
					dont_cont = 1;
				}
			}
			if (dont_cont == 0){
			stringstream n1;
			n1 << fixed << setprecision(pose-2) << msg_str.substr(0,pose);
			num1 = stof(n1.str());
			//ROS_INFO_STREAM(num1);
			stringstream bb;
			bb << num1 << ' ';
			num_len = bb.str();
			stringstream n2;
			n2 << fixed << setprecision(pose-2) << msg_str.substr(pose+1,(msg_str.length()-(pose+1)));
			num2 = stof(n2.str());
			//num2 = stof(msg_str);
			//ROS_INFO_STREAM(num2);			
			arith_pub.publish(math);
			stringstream cc;
			cc << msg->message[num_len.length()-1] << ' ';
			oper = cc.str();
			oper = oper[0];
			
			//ROS_INFO_STREAM(oper);
			math.oper_type = cc.str();
			
			if (oper == "+"){
				math.answer = num1+num2;
				time_answered = ros::Time::now();
				ROS_INFO_STREAM(time_answered);
			}
			else if(oper == "-"){
				math.answer = num1-num2;
				time_answered = ros::Time::now();
			}
			else if(oper == "*"){
				math.answer = num1*num2;
				time_answered = ros::Time::now();
			}
			else if(oper == "/"){
				float answer = float(num1/num2);
				math.answer = answer;
				time_answered = ros::Time::now();
			}
			math.time_answered = float(time_answered.toSec());
			//ROS_INFO_STREAM(math.time_answered);
			math.time_received = float(time_received.toSec());
			//ROS_INFO_STREAM(math.time_received);
			process_time = time_answered-time_received;
			math.process_time = float(process_time.toSec());
			arith_pub.publish(math);
			}
		
		
	}	
		
	

}


int main(int argc, char **argv){

	ros::init(argc, argv, "arithmetic_node");
	ros::init(argc, argv, "message_ui");
	ros::NodeHandle n;
	
	//Publish
	arith_pub = n.advertise<arithmetic_node::arithmetic_reply>("arithmetic_reply", 1000);
	ros::Rate loop_rate(20);
	
	//Subscriber
	arith_sub = n.subscribe("sent_msg", 1000, arith_callback);
	ros::spin();

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



