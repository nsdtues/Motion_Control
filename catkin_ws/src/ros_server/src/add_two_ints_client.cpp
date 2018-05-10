#include "ros/ros.h"
#include "ros_server/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  double timeout_;
  ros::NodeHandle n;
  ros::NodeHandle nh_priv("~");

  nh_priv.param("timeout", timeout_, 0.0);

  ros::ServiceClient client = n.serviceClient<ros_server::AddTwoInts>("add_two_ints");
  ros_server::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  ROS_INFO("timeout_:%lf", timeout_);

  if(client.waitForExistence(ros::Duration(timeout_)) == true){
      ROS_INFO("server start.");
  }else{
      ROS_INFO("server fail.");
  }

  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
