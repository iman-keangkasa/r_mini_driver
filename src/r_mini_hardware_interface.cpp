#include "r_mini_driver/r_mini_hardware_interface.h"

RminiHW::RminiHW (double period, const std::string port_id, const uint32_t baud_rate)
  : period_(period),
  port_id_(port_id),
  baudrate_(baud_rate),
  //node_handle_(""),
  //priv_node_handle_("~"),
  id_shoulder_{1,2,3},
  id_wrist_{4,5,6},
  counter_(0)
{
  writeFirst();
  int joint_state_cnt = 0;
  int joint_pos_cnt =0; 
/*  std::string joint_names[JOINT_NUM] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

  //getting all the joints registered for joint_state_interface

  for(joint_state_cnt; joint_state_cnt < JOINT_NUM; ++joint_state_cnt)
  {
    ROS_INFO_STREAM(joint_names[joint_state_cnt]);
    hardware_interface::JointStateHandle state_handle(joint_names[joint_state_cnt], &pos_[joint_state_cnt], &vel_[joint_state_cnt], &effort_[joint_state_cnt]);
    joint_state_interface_.registerHandle(state_handle);
    
    //getting the id into an array
    id_array_[joint_state_cnt] = joint_state_cnt;
  }
*/

  for(std::map<std::string, uint32_t>::iterator it = dynamixel_.begin(); it != dynamixel_.end(); ++it)
  {
    ROS_INFO_STREAM(it->first);
    hardware_interface::JointStateHandle state_handle(it->first, &pos_[joint_state_cnt], &vel_[joint_state_cnt], &effort_[joint_state_cnt]);
    joint_state_interface_.registerHandle(state_handle);
    
    //getting the id into an array
    id_array_[joint_state_cnt] = it->second;
    joint_state_cnt++;

  }

  registerInterface(&joint_state_interface_);

  joint_state_cnt = 0;
  //getting all the joints registered for joint_position_inteface (command)
/*  for(joint_state_cnt; joint_state_cnt < JOINT_NUM; ++joint_state_cnt)
  {
    ROS_INFO_STREAM(joint_names[joint_state_cnt]);
    hardware_interface::JointHandle joint_pos_handle(joint_state_interface_.getHandle(joint_names[joint_state_cnt]), &cmd_[joint_state_cnt]);
    joint_pos_interface_.registerHandle(joint_pos_handle);
    }
*/

  for(std::map<std::string, uint32_t>::iterator it = dynamixel_.begin(); it != dynamixel_.end(); ++it)
  {
    ROS_INFO_STREAM(it->first);
    hardware_interface::JointHandle joint_pos_handle(joint_state_interface_.getHandle(it->first), &cmd_[joint_pos_cnt]);
    joint_pos_interface_.registerHandle(joint_pos_handle);
  }

  
  registerInterface(&joint_pos_interface_);


}

//RminiHW::~RminiHW() {ROS_INFO("Destructor Called");}

//void RminiHW::update(void){}

void RminiHW::write(void)
{
  bool result (false);
  const char* log = NULL;

  for (int index = 0; index < 3; index++)
    dynamixel_position123_[index] = dxl_wb_.convertRadian2Value(id_shoulder_[index], cmd_[index]);
  for (int index = 0; index < 3; index++)
    dynamixel_position456_[index] = dxl_wb_.convertRadian2Value(id_wrist_[index], cmd_[index+3]);
    
  result = dxl_wb_.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION123, id_shoulder_, 3, dynamixel_position123_, 1, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
    
  result = dxl_wb_.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION456, id_wrist_, 3, dynamixel_position456_, 1, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void RminiHW::read(void)
{
  bool result = false;
  const char* log = NULL;

  if (dxl_wb_.getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_.syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT123,
                                  id_shoulder_,
                                  3,
                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_.syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT456,
                                  id_wrist_,
                                  3,
                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
      
    result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT123,
                                                    id_shoulder_,
                                                    3,
                                                    control_items_["Present_Current123"]->address,
                                                    control_items_["Present_Current123"]->data_length,
                                                    get_current123_,
                                                    &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT456,
                                                    id_wrist_,
                                                    3,
                                                    control_items_["Present_Current456"]->address,
                                                    control_items_["Present_Current456"]->data_length,
                                                    get_current456_,
                                                    &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT123,
                                                    id_shoulder_,
                                                    3,
                                                    control_items_["Present_Velocity123"]->address,
                                                    control_items_["Present_Velocity123"]->data_length,
                                                    get_velocity123_,
                                                    &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT456,
                                                    id_wrist_,
                                                    3,
                                                    control_items_["Present_Velocity456"]->address,
                                                    control_items_["Present_Velocity456"]->data_length,
                                                    get_velocity456_,
                                                    &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT123,
                                                    id_shoulder_,
                                                    3,
                                                    control_items_["Present_Position123"]->address,
                                                    control_items_["Present_Position123"]->data_length,
                                                    get_position123_,
                                                    &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_.getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT456,
                                                    id_wrist_,
                                                    3,
                                                    control_items_["Present_Position456"]->address,
                                                    control_items_["Present_Position456"]->data_length,
                                                    get_position456_,
                                                    &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
  }
   //Converting values to radian
  //populate pos_ with values for joint1 to joint3
  for (int index; index < 3; ++index)
  {
    pos_[index] = dxl_wb_.convertValue2Radian(id_shoulder_[index], get_position123_[index]);
  }
  
  //populate pos_ with values for joint4 to joint6
  for (int index; index < 3; ++index)
  {
    pos_[index+3] = dxl_wb_.convertValue2Radian(id_wrist_[index], get_position456_[index]);
  }
  
  //Converting values to velocity
  for (int index; index < 3; ++index)
  {
    vel_[index] = dxl_wb_.convertValue2Velocity(id_shoulder_[index], get_velocity123_[index]);
  }

  for (int index; index < 3; ++index)
  {
    vel_[index+3] = dxl_wb_.convertValue2Velocity(id_wrist_[index], get_velocity456_[index]);
  }

  //converting values to current [IMAN] [TODO] attempt at current to torque mapping 
  for (int index; index < 3; ++index)
  {
    effort_[index] = dxl_wb_.convertValue2Current(id_shoulder_[index], get_current123_[index]);
  }

  for (int index; index < 3; ++index)
  {
    effort_[index+3] = dxl_wb_.convertValue2Current(id_wrist_[index], get_current456_[index]);
  }

  
  //initialize cmd_ to current position so 
  // the robot would not jump to default pose
  if(counter_==0) 
  {
    for(int joint_cnt=0; joint_cnt < JOINT_NUM; ++joint_cnt)
      cmd_[joint_cnt] = pos_[joint_cnt];
  }
  counter_++;
}

void RminiHW::writeFirst(void) 
{
  //initialize dynamixel workbench 
  bool result = false;
  const char* log;
  
  result = dxl_wb_.init(port_id_.c_str(), baudrate_, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
  ROS_ERROR("[DEBUG]Finish initializsing dynamixel");

  //get Dynamixelinfo 
  ros::param::param<std::string>("~dynamixel_info", yaml_file_, "p");
  ROS_INFO("[DEBUG:%s",yaml_file_.c_str()); 
  //here dynamixel_ and dynamixel_info_ is set
  result = getDynamixelsInfo();
  if(result == false)
  {
    ROS_ERROR("DEBUG HERE: getDynamixelsInfo");
  }

  result = loadDynamixels();
  if(result == false)
  {
    ROS_ERROR("DEBUG HERE: getDynamixelsInfo");
  }
  
  result = initDynamixels();
  if(result == false)
  {
    ROS_ERROR("DEBUG HERE: getDynamixelsInfo");
  }

  result = initControlItems();
  if(result == false)
  {
    ROS_ERROR("DEBUG HERE: getDynamixelsInfo");
  }

  result = initSDKHandlers();
  if(result == false)
  {
    ROS_ERROR("DEBUG HERE: getDynamixelsInfo");
  }
}

bool RminiHW::getDynamixelsInfo(void)
{
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file_.c_str());

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}

bool RminiHW::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_.ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {      
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}

bool RminiHW::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_.torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_.itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
          //else ROS_INFO("Succeed writing value[%d] on items[%s] to Dynamixel[Name: %s, ID: %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);

        }
      }
    }

    dxl_wb_.torqueOn((uint8_t)dxl.second);
  }

  return true;
}

bool RminiHW::initControlItems(void)
{
  bool result = false;
  const char* log = NULL;

  auto it123 = dynamixel_["joint1"];
  auto it456 = dynamixel_["joint4"];
  const ControlItem *goal_position123 = dxl_wb_.getItemInfo(it123, "Goal_Position");
  if (goal_position123 == NULL) return false;
  
  const ControlItem *goal_position456 = dxl_wb_.getItemInfo(it456, "Goal_Position");
  if (goal_position456 == NULL) return false;

  const ControlItem *goal_velocity123 = dxl_wb_.getItemInfo(it123, "Goal_Velocity");
  if (goal_velocity123 == NULL)  goal_velocity123 = dxl_wb_.getItemInfo(it123, "Moving_Speed");
  if (goal_velocity123 == NULL)  return false;

  const ControlItem *goal_velocity456 = dxl_wb_.getItemInfo(it456, "Goal_Velocity");
  if (goal_velocity456 == NULL)  goal_velocity456 = dxl_wb_.getItemInfo(it456, "Moving_Speed");
  if (goal_velocity456 == NULL)  return false;

  const ControlItem *present_position123 = dxl_wb_.getItemInfo(it123, "Present_Position");
  if (present_position123 == NULL) return false;

  const ControlItem *present_position456 = dxl_wb_.getItemInfo(it456, "Present_Position");
  if (present_position456 == NULL) return false;

  const ControlItem *present_velocity123 = dxl_wb_.getItemInfo(it123, "Present_Velocity");
  if (present_velocity123 == NULL)  present_velocity123 = dxl_wb_.getItemInfo(it123, "Present_Speed");
  if (present_velocity123 == NULL) return false;

  const ControlItem *present_velocity456 = dxl_wb_.getItemInfo(it456, "Present_Velocity");
  if (present_velocity456 == NULL)  present_velocity456 = dxl_wb_.getItemInfo(it456, "Present_Speed");
  if (present_velocity456 == NULL) return false;

  const ControlItem *present_current123 = dxl_wb_.getItemInfo(it123, "Present_Current");
  if (present_current123 == NULL)  present_current123 = dxl_wb_.getItemInfo(it123, "Present_Load");
  if (present_current123 == NULL) return false;

  const ControlItem *present_current456 = dxl_wb_.getItemInfo(it456, "Present_Current");
  if (present_current456 == NULL)  present_current456 = dxl_wb_.getItemInfo(it456, "Present_Load");
  if (present_current456 == NULL) return false;

  control_items_["Goal_Position123"] = goal_position123;
  control_items_["Goal_Velocity123"] = goal_velocity123;

  control_items_["Present_Position123"] = present_position123;
  control_items_["Present_Velocity123"] = present_velocity123;
  control_items_["Present_Current123"] = present_current123;

  control_items_["Goal_Position456"] = goal_position456;
  control_items_["Goal_Velocity456"] = goal_velocity456;

  control_items_["Present_Position456"] = present_position456;
  control_items_["Present_Velocity456"] = present_velocity456;
  control_items_["Present_Current456"] = present_current456;

  return true;
}

bool RminiHW::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();
  /*Adding shoulder joints (joint1, joint2, joint3) write handler */
  result = dxl_wb_.addSyncWriteHandler(control_items_["Goal_Position123"]->address, control_items_["Goal_Position123"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_.addSyncWriteHandler(control_items_["Goal_Velocity123"]->address, control_items_["Goal_Velocity123"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  /*Adding wrist joints (joint4, joint5, joint6) write handler */
  result = dxl_wb_.addSyncWriteHandler(control_items_["Goal_Position456"]->address, control_items_["Goal_Position456"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  result = dxl_wb_.addSyncWriteHandler(control_items_["Goal_Velocity456"]->address, control_items_["Goal_Velocity456"]->data_length, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }
  else
  {
    ROS_INFO("%s", log);
  }

  if (dxl_wb_.getProtocolVersion() == 2.0f)
  {  
    uint16_t start_address123 = std::min(control_items_["Present_Position123"]->address, control_items_["Present_Current123"]->address);

    /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */    
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length123 = control_items_["Present_Position123"]->data_length + control_items_["Present_Velocity123"]->data_length + control_items_["Present_Current123"]->data_length+2;

    result = dxl_wb_.addSyncReadHandler(start_address123,
                                          read_length123,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    } 
    
    uint16_t start_address456 = std::min(control_items_["Present_Position456"]->address, control_items_["Present_Current456"]->address);

    /* 
      As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.
    */    
    // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
    uint16_t read_length456 = control_items_["Present_Position456"]->data_length + control_items_["Present_Velocity123"]->data_length + control_items_["Present_Current456"]->data_length+2;

    result = dxl_wb_.addSyncReadHandler(start_address456,
                                          read_length456,
                                          &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }

  }

  return result;
}
