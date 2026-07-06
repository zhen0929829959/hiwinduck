#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to hiwin_interfaces__srv__Hiwinmodbus_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Hiwinmodbus_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub holding: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub type_: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub vel: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acc: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub digital_output: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub onoff: i32,

    /// Digital_output[DO_Num, 0or1]
    pub pose: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub circ_s: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub circ_end: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dir: i32,

}



impl Default for Hiwinmodbus_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Hiwinmodbus_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Hiwinmodbus_Request {
  type RmwMsg = super::srv::rmw::Hiwinmodbus_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mode: msg.mode.as_str().into(),
        holding: msg.holding,
        type_: msg.type_,
        vel: msg.vel,
        acc: msg.acc,
        digital_output: msg.digital_output,
        onoff: msg.onoff,
        pose: msg.pose.into(),
        circ_s: msg.circ_s.into(),
        circ_end: msg.circ_end.into(),
        joint: msg.joint,
        dir: msg.dir,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mode: msg.mode.as_str().into(),
      holding: msg.holding,
      type_: msg.type_,
      vel: msg.vel,
      acc: msg.acc,
      digital_output: msg.digital_output,
      onoff: msg.onoff,
        pose: msg.pose.as_slice().into(),
        circ_s: msg.circ_s.as_slice().into(),
        circ_end: msg.circ_end.as_slice().into(),
      joint: msg.joint,
      dir: msg.dir,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      mode: msg.mode.to_string(),
      holding: msg.holding,
      type_: msg.type_,
      vel: msg.vel,
      acc: msg.acc,
      digital_output: msg.digital_output,
      onoff: msg.onoff,
      pose: msg.pose
          .into_iter()
          .collect(),
      circ_s: msg.circ_s
          .into_iter()
          .collect(),
      circ_end: msg.circ_end
          .into_iter()
          .collect(),
      joint: msg.joint,
      dir: msg.dir,
    }
  }
}


// Corresponds to hiwin_interfaces__srv__Hiwinmodbus_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Hiwinmodbus_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub arm_state: i32,

}



impl Default for Hiwinmodbus_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Hiwinmodbus_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Hiwinmodbus_Response {
  type RmwMsg = super::srv::rmw::Hiwinmodbus_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        arm_state: msg.arm_state,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      arm_state: msg.arm_state,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      arm_state: msg.arm_state,
    }
  }
}


// Corresponds to hiwin_interfaces__srv__RobotCommand_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotCommand_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub do_timer: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub holding: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cmd_mode: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub cmd_type: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub acceleration: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tool: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub base: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub base_num: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub tool_num: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub digital_input_pin: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub digital_output_pin: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub digital_output_cmd: u8,

    /// for POSE_CMD
    pub pose: geometry_msgs::msg::Twist,

    /// for JOINTS_CMD
    pub joints: [f64; 6],


    // This member is not documented.
    #[allow(missing_docs)]
    pub circ_s: Vec<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub circ_end: Vec<f64>,

    /// for JOG mode
    pub jog_joint: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub jog_dir: i8,

    /// for FLANGE_MOVE mode
    pub move_dir: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub move_dis: f64,

}

impl RobotCommand_Request {
    /// mode
    pub const EXCITE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PTP: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LINE: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CIRC: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DIGITAL_OUTPUT: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const HOME: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const JOG: u8 = 7;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CHECK_JOINT: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CHECK_POSE: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLOSE: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const WAITING: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const READ_DI: u8 = 12;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SET_BASE: u8 = 13;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SET_TOOL: u8 = 14;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MOTION_STOP: u8 = 15;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FLANGE_MOVE: u8 = 16;

    /// cmd type
    pub const JOINTS_CMD: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const POSE_CMD: u8 = 1;

    /// digital_output_cmd
    pub const DIGITAL_ON: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DIGITAL_OFF: u8 = 0;

}


impl Default for RobotCommand_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::RobotCommand_Request::default())
  }
}

impl rosidl_runtime_rs::Message for RobotCommand_Request {
  type RmwMsg = super::srv::rmw::RobotCommand_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        do_timer: msg.do_timer,
        holding: msg.holding,
        cmd_mode: msg.cmd_mode,
        cmd_type: msg.cmd_type,
        velocity: msg.velocity,
        acceleration: msg.acceleration,
        tool: msg.tool,
        base: msg.base,
        base_num: msg.base_num,
        tool_num: msg.tool_num,
        digital_input_pin: msg.digital_input_pin,
        digital_output_pin: msg.digital_output_pin,
        digital_output_cmd: msg.digital_output_cmd,
        pose: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        joints: msg.joints,
        circ_s: msg.circ_s.into(),
        circ_end: msg.circ_end.into(),
        jog_joint: msg.jog_joint,
        jog_dir: msg.jog_dir,
        move_dir: msg.move_dir.as_str().into(),
        move_dis: msg.move_dis,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      do_timer: msg.do_timer,
      holding: msg.holding,
      cmd_mode: msg.cmd_mode,
      cmd_type: msg.cmd_type,
      velocity: msg.velocity,
      acceleration: msg.acceleration,
      tool: msg.tool,
      base: msg.base,
      base_num: msg.base_num,
      tool_num: msg.tool_num,
      digital_input_pin: msg.digital_input_pin,
      digital_output_pin: msg.digital_output_pin,
      digital_output_cmd: msg.digital_output_cmd,
        pose: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        joints: msg.joints,
        circ_s: msg.circ_s.as_slice().into(),
        circ_end: msg.circ_end.as_slice().into(),
      jog_joint: msg.jog_joint,
      jog_dir: msg.jog_dir,
        move_dir: msg.move_dir.as_str().into(),
      move_dis: msg.move_dis,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      do_timer: msg.do_timer,
      holding: msg.holding,
      cmd_mode: msg.cmd_mode,
      cmd_type: msg.cmd_type,
      velocity: msg.velocity,
      acceleration: msg.acceleration,
      tool: msg.tool,
      base: msg.base,
      base_num: msg.base_num,
      tool_num: msg.tool_num,
      digital_input_pin: msg.digital_input_pin,
      digital_output_pin: msg.digital_output_pin,
      digital_output_cmd: msg.digital_output_cmd,
      pose: geometry_msgs::msg::Twist::from_rmw_message(msg.pose),
      joints: msg.joints,
      circ_s: msg.circ_s
          .into_iter()
          .collect(),
      circ_end: msg.circ_end
          .into_iter()
          .collect(),
      jog_joint: msg.jog_joint,
      jog_dir: msg.jog_dir,
      move_dir: msg.move_dir.to_string(),
      move_dis: msg.move_dis,
    }
  }
}


// Corresponds to hiwin_interfaces__srv__RobotCommand_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotCommand_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub arm_state: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub digital_state: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_position: Vec<f64>,

}

impl RobotCommand_Response {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const IDLE: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RUNNING: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const HOLD: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DELAY: u8 = 4;

}


impl Default for RobotCommand_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::RobotCommand_Response::default())
  }
}

impl rosidl_runtime_rs::Message for RobotCommand_Response {
  type RmwMsg = super::srv::rmw::RobotCommand_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        arm_state: msg.arm_state,
        digital_state: msg.digital_state,
        current_position: msg.current_position.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      arm_state: msg.arm_state,
      digital_state: msg.digital_state,
        current_position: msg.current_position.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      arm_state: msg.arm_state,
      digital_state: msg.digital_state,
      current_position: msg.current_position
          .into_iter()
          .collect(),
    }
  }
}






#[link(name = "hiwin_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__hiwin_interfaces__srv__Hiwinmodbus() -> *const std::ffi::c_void;
}

// Corresponds to hiwin_interfaces__srv__Hiwinmodbus
#[allow(missing_docs, non_camel_case_types)]
pub struct Hiwinmodbus;

impl rosidl_runtime_rs::Service for Hiwinmodbus {
    type Request = Hiwinmodbus_Request;
    type Response = Hiwinmodbus_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__hiwin_interfaces__srv__Hiwinmodbus() }
    }
}




#[link(name = "hiwin_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__hiwin_interfaces__srv__RobotCommand() -> *const std::ffi::c_void;
}

// Corresponds to hiwin_interfaces__srv__RobotCommand
#[allow(missing_docs, non_camel_case_types)]
pub struct RobotCommand;

impl rosidl_runtime_rs::Service for RobotCommand {
    type Request = RobotCommand_Request;
    type Response = RobotCommand_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__hiwin_interfaces__srv__RobotCommand() }
    }
}


