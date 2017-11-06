#include <ros/ros.h>
#include <chatbot_node/reply_msg.h>
#include <message_ui/sent_msg.h>
#include <sstream>

using namespace std;
ros::Publisher chatbot_pub;
string name;

//Add your code here
void chatbotCallback(const message_ui::sent_msg::ConstPtr& msg){
  chatbot_node::reply_msg reply;
  stringstream ss;
  if (int(msg->message[0]) > 64 && int(msg->message[0]) < 91){
    if (msg->message == "Hello"){
       ss << "Hello " + name << ' ';
       reply.message = ss.str();
       chatbot_pub.publish(reply);
    }
    else if (msg->message == "What is your name?"){
       ss << "My name is MRSD Siri" << ' ';
       reply.message = ss.str();
       chatbot_pub.publish(reply);
    }
    else if (msg->message == "How are you?"){
       ss << "I am fine, thank you." << ' ';
       reply.message = ss.str();
       chatbot_pub.publish(reply);
    }
  }
  //ROS_INFO("chatbot %i",int(msg->message[0]));
  
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "chatbot_node");
  ros::init(argc, argv, "message_ui");
  ros::NodeHandle n;
  n.getParam("name", name);

  //Publish reply_msg
  chatbot_pub = n.advertise<chatbot_node::reply_msg>("reply_msg",1000);
  ros::Rate loop_rate(20);
  
  
  //Subscribe sent_msg
  ros::Subscriber chatbot_sub = n.subscribe("sent_msg",1000,chatbotCallback);
  ros::spin();

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
