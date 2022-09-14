#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

//MUST FOLLOW THE ORDER IN initSDKHandlers()
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION123 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY123 1
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION456 2
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY456 3
//#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT123 4
//#define SYNC_WRITE_HANDLER_FOR_GOAL_CURRENT456 5

//for protocol 2.0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT123 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT456 1

#define JOINT_NUM 6

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class RminiHW : public hardware_interface::RobotHW
{
  public:
   RminiHW(double period, const std::string port_id, const uint32_t baud_rate);
//   ~RminiHW();
   void update(void);
   void write(void);
   void read(void);
   void writeFirst(void);

   inline ros::Duration getPeriod()
   {
     return ros::Duration(period_);
   }
   bool getDynamixelsInfo(void);
   bool loadDynamixels(void);
   bool initDynamixels(void);
   bool initControlItems(void);
   bool initSDKHandlers(void);
  private:
   //ros::NodeHandle node_handle_;
   //ros::NodeHandle priv_node_handle_;

   double period_;
   ros::Time time_now_;
   ros::Time time_old_;

   std::string port_id_;
   uint32_t baudrate_;

   int counter_;

   hardware_interface::JointStateInterface joint_state_interface_;
   hardware_interface::PositionJointInterface joint_pos_interface_;

   double cmd_[JOINT_NUM];
   double cmd_vel_[JOINT_NUM];
   double cmd_current_[JOINT_NUM];
   double pos_[JOINT_NUM];
   double vel_[JOINT_NUM];
   double effort_[JOINT_NUM];

   DynamixelWorkbench dxl_wb_;
   std::map<std::string, uint32_t> dynamixel_; //dynamixel name and ID
   std::map<std::string, const ControlItem*> control_items_;
   std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
   std::string yaml_file_;
   uint8_t id_shoulder_[3];
   uint8_t id_wrist_[3];
   uint8_t id_array_[JOINT_NUM];
   int32_t dynamixel_position123_[3];
   int32_t dynamixel_position456_[3];
   int32_t get_current123_[3];
   int32_t get_current456_[3];
   int32_t get_velocity123_[3];
   int32_t get_velocity456_[3];
   int32_t get_position123_[3];
   int32_t get_position456_[3];


};
