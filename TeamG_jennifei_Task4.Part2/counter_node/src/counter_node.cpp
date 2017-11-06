#include <ros/ros.h>
#include <chatbot_node/reply_msg.h>
#include <message_ui/sent_msg.h>
#include <counter_node/counter.h>
#include <sstream>
#include <arithmetic_node/arithmetic_reply.h>

using namespace std;
int num_reply_msg = 0;
int num_sent_msg = 0;

ros::Time last_sent_msg_time;
ros::Time last_reply_msg_time;
ros::Duration time_since_reply;
ros::Duration time_since_user;

ros::Subscriber reply_msg_sub;
ros::Subscriber arithmetic_reply_msg_sub;
ros::Subscriber sent_msg_sub;

ros::Publisher counter_pub;
ros::ServiceServer counter_serv;

void sent_msg_callback(const message_ui::sent_msg msg)
{
	num_sent_msg++;
	last_sent_msg_time = msg.header.stamp;
}

void reply_msg_callback(const chatbot_node::reply_msg msg)
{
	num_reply_msg++;
	last_reply_msg_time = msg.header.stamp;
	
}

bool service_callback(counter_node::counter::Request& req, counter_node::counter::Response& resp)
{

	if(req.req_id == 0){
		//resp.reply = 5.0;
		resp.reply = float(num_reply_msg+num_sent_msg);
		
	}
	else if(req.req_id == 1){
		resp.reply = float(num_reply_msg);
		
	}
	else if(req.req_id == 2){
		resp.reply = float(num_sent_msg);
		
	}
	else if(req.req_id == 3){
		time_since_reply = ros::Time::now() - (last_reply_msg_time);
		resp.reply = float(time_since_reply.toSec());
		
	}		
	else if(req.req_id == 4){
		time_since_user = ros::Time::now() - (last_sent_msg_time);
		resp.reply = float(time_since_user.toSec());
		
	}
	
	return true;
	
}
 void arithmetic_reply_msg_callback(const arithmetic_node::arithmetic_reply msg)
 {
 	num_reply_msg++;
 	last_reply_msg_time = msg.header.stamp;
 }

int main(int argc, char **argv) {

  ros::init(argc, argv, "counter_node");
  ros::NodeHandle n;
  
  
  //subscibe
  reply_msg_sub = n.subscribe("reply_msg", 1000, reply_msg_callback);
  sent_msg_sub = n.subscribe("sent_msg", 1000, sent_msg_callback);
  arithmetic_reply_msg_sub = n.subscribe("arithmetic_reply", 1000, arithmetic_reply_msg_callback);
  
  //service
  counter_serv = n.advertiseService("message_counter", service_callback);
  ros::spin();

  ros::Rate loop_rate(20);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
