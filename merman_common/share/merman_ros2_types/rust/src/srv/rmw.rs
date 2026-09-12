#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "merman_ros2_types__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__merman_ros2_types__srv__PoseArrived_Request() -> *const std::ffi::c_void;
}

#[link(name = "merman_ros2_types__rosidl_generator_c")]
extern "C" {
    fn merman_ros2_types__srv__PoseArrived_Request__init(msg: *mut PoseArrived_Request) -> bool;
    fn merman_ros2_types__srv__PoseArrived_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PoseArrived_Request>, size: usize) -> bool;
    fn merman_ros2_types__srv__PoseArrived_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PoseArrived_Request>);
    fn merman_ros2_types__srv__PoseArrived_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PoseArrived_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PoseArrived_Request>) -> bool;
}

// Corresponds to merman_ros2_types__srv__PoseArrived_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseArrived_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose_id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub play_audio: bool,

}



impl Default for PoseArrived_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !merman_ros2_types__srv__PoseArrived_Request__init(&mut msg as *mut _) {
        panic!("Call to merman_ros2_types__srv__PoseArrived_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PoseArrived_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__srv__PoseArrived_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__srv__PoseArrived_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__srv__PoseArrived_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PoseArrived_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PoseArrived_Request where Self: Sized {
  const TYPE_NAME: &'static str = "merman_ros2_types/srv/PoseArrived_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__merman_ros2_types__srv__PoseArrived_Request() }
  }
}


#[link(name = "merman_ros2_types__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__merman_ros2_types__srv__PoseArrived_Response() -> *const std::ffi::c_void;
}

#[link(name = "merman_ros2_types__rosidl_generator_c")]
extern "C" {
    fn merman_ros2_types__srv__PoseArrived_Response__init(msg: *mut PoseArrived_Response) -> bool;
    fn merman_ros2_types__srv__PoseArrived_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PoseArrived_Response>, size: usize) -> bool;
    fn merman_ros2_types__srv__PoseArrived_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PoseArrived_Response>);
    fn merman_ros2_types__srv__PoseArrived_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PoseArrived_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PoseArrived_Response>) -> bool;
}

// Corresponds to merman_ros2_types__srv__PoseArrived_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseArrived_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for PoseArrived_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !merman_ros2_types__srv__PoseArrived_Response__init(&mut msg as *mut _) {
        panic!("Call to merman_ros2_types__srv__PoseArrived_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PoseArrived_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__srv__PoseArrived_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__srv__PoseArrived_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__srv__PoseArrived_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PoseArrived_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PoseArrived_Response where Self: Sized {
  const TYPE_NAME: &'static str = "merman_ros2_types/srv/PoseArrived_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__merman_ros2_types__srv__PoseArrived_Response() }
  }
}






#[link(name = "merman_ros2_types__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__merman_ros2_types__srv__PoseArrived() -> *const std::ffi::c_void;
}

// Corresponds to merman_ros2_types__srv__PoseArrived
#[allow(missing_docs, non_camel_case_types)]
pub struct PoseArrived;

impl rosidl_runtime_rs::Service for PoseArrived {
    type Request = PoseArrived_Request;
    type Response = PoseArrived_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__merman_ros2_types__srv__PoseArrived() }
    }
}


