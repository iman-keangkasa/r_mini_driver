#include "r_mini_driver/r_mini_hardware_interface.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "r_mini_driver_node");
  ros::NodeHandle node_handle;
  
  std::string port_name;
  uint32_t baud_rate;
  double period;
  ros::param::param<double>("~period", period, 0.001);

  if (argc < 2)
  {
    ROS_ERROR("Please set 'port_name' and 'baud_rate' argmuments for connected Dynamixels");
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
  }

  RminiHW r_mini(period, port_name, baud_rate);
  controller_manager::ControllerManager cm(&r_mini, node_handle);
  ros::Rate rate (1.0/r_mini.getPeriod().toSec());
  ros::AsyncSpinner spinner (2);
  spinner.start();

  r_mini.writeFirst();
  
  while(ros::ok())
  {
    r_mini.read();
    cm.update(ros::Time::now(), r_mini.getPeriod());
    r_mini.write();

    rate.sleep();
  }
  
  //spinner.stop();
  return 0;
}


  

