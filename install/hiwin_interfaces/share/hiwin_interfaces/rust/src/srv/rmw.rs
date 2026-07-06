#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "hiwin_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hiwin_interfaces__srv__Hiwinmodbus_Request() -> *const std::ffi::c_void;
}

#[link(name = "hiwin_interfaces__rosidl_generator_c")]
extern "C" {
    fn hiwin_interfaces__srv__Hiwinmodbus_Request__init(msg: *mut Hiwinmodbus_Request) -> bool;
    fn hiwin_interfaces__srv__Hiwinmodbus_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Hiwinmodbus_Request>, size: usize) -> bool;
    fn hiwin_interfaces__srv__Hiwinmodbus_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Hiwinmodbus_Request>);
    fn hiwin_interfaces__srv__Hiwinmodbus_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Hiwinmodbus_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Hiwinmodbus_Request>) -> bool;
}

// Corresponds to hiwin_interfaces__srv__Hiwinmodbus_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Hiwinmodbus_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: rosidl_runtime_rs::String,


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
    pub pose: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub circ_s: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub circ_end: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub joint: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub dir: i32,

}



impl Default for Hiwinmodbus_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hiwin_interfaces__srv__Hiwinmodbus_Request__init(&mut msg as *mut _) {
        panic!("Call to hiwin_interfaces__srv__Hiwinmodbus_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Hiwinmodbus_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__Hiwinmodbus_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__Hiwinmodbus_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__Hiwinmodbus_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Hiwinmodbus_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Hiwinmodbus_Request where Self: Sized {
  const TYPE_NAME: &'static str = "hiwin_interfaces/srv/Hiwinmodbus_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hiwin_interfaces__srv__Hiwinmodbus_Request() }
  }
}


#[link(name = "hiwin_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hiwin_interfaces__srv__Hiwinmodbus_Response() -> *const std::ffi::c_void;
}

#[link(name = "hiwin_interfaces__rosidl_generator_c")]
extern "C" {
    fn hiwin_interfaces__srv__Hiwinmodbus_Response__init(msg: *mut Hiwinmodbus_Response) -> bool;
    fn hiwin_interfaces__srv__Hiwinmodbus_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Hiwinmodbus_Response>, size: usize) -> bool;
    fn hiwin_interfaces__srv__Hiwinmodbus_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Hiwinmodbus_Response>);
    fn hiwin_interfaces__srv__Hiwinmodbus_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Hiwinmodbus_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Hiwinmodbus_Response>) -> bool;
}

// Corresponds to hiwin_interfaces__srv__Hiwinmodbus_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Hiwinmodbus_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub arm_state: i32,

}



impl Default for Hiwinmodbus_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hiwin_interfaces__srv__Hiwinmodbus_Response__init(&mut msg as *mut _) {
        panic!("Call to hiwin_interfaces__srv__Hiwinmodbus_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Hiwinmodbus_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__Hiwinmodbus_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__Hiwinmodbus_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__Hiwinmodbus_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Hiwinmodbus_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Hiwinmodbus_Response where Self: Sized {
  const TYPE_NAME: &'static str = "hiwin_interfaces/srv/Hiwinmodbus_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hiwin_interfaces__srv__Hiwinmodbus_Response() }
  }
}


#[link(name = "hiwin_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hiwin_interfaces__srv__RobotCommand_Request() -> *const std::ffi::c_void;
}

#[link(name = "hiwin_interfaces__rosidl_generator_c")]
extern "C" {
    fn hiwin_interfaces__srv__RobotCommand_Request__init(msg: *mut RobotCommand_Request) -> bool;
    fn hiwin_interfaces__srv__RobotCommand_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotCommand_Request>, size: usize) -> bool;
    fn hiwin_interfaces__srv__RobotCommand_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotCommand_Request>);
    fn hiwin_interfaces__srv__RobotCommand_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotCommand_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotCommand_Request>) -> bool;
}

// Corresponds to hiwin_interfaces__srv__RobotCommand_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub pose: geometry_msgs::msg::rmw::Twist,

    /// for JOINTS_CMD
    pub joints: [f64; 6],


    // This member is not documented.
    #[allow(missing_docs)]
    pub circ_s: rosidl_runtime_rs::Sequence<f64>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub circ_end: rosidl_runtime_rs::Sequence<f64>,

    /// for JOG mode
    pub jog_joint: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub jog_dir: i8,

    /// for FLANGE_MOVE mode
    pub move_dir: rosidl_runtime_rs::String,


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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hiwin_interfaces__srv__RobotCommand_Request__init(&mut msg as *mut _) {
        panic!("Call to hiwin_interfaces__srv__RobotCommand_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotCommand_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__RobotCommand_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__RobotCommand_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__RobotCommand_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotCommand_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotCommand_Request where Self: Sized {
  const TYPE_NAME: &'static str = "hiwin_interfaces/srv/RobotCommand_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hiwin_interfaces__srv__RobotCommand_Request() }
  }
}


#[link(name = "hiwin_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hiwin_interfaces__srv__RobotCommand_Response() -> *const std::ffi::c_void;
}

#[link(name = "hiwin_interfaces__rosidl_generator_c")]
extern "C" {
    fn hiwin_interfaces__srv__RobotCommand_Response__init(msg: *mut RobotCommand_Response) -> bool;
    fn hiwin_interfaces__srv__RobotCommand_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotCommand_Response>, size: usize) -> bool;
    fn hiwin_interfaces__srv__RobotCommand_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotCommand_Response>);
    fn hiwin_interfaces__srv__RobotCommand_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotCommand_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotCommand_Response>) -> bool;
}

// Corresponds to hiwin_interfaces__srv__RobotCommand_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
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
    pub current_position: rosidl_runtime_rs::Sequence<f64>,

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
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hiwin_interfaces__srv__RobotCommand_Response__init(&mut msg as *mut _) {
        panic!("Call to hiwin_interfaces__srv__RobotCommand_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotCommand_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__RobotCommand_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__RobotCommand_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hiwin_interfaces__srv__RobotCommand_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotCommand_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotCommand_Response where Self: Sized {
  const TYPE_NAME: &'static str = "hiwin_interfaces/srv/RobotCommand_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hiwin_interfaces__srv__RobotCommand_Response() }
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


