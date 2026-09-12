#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "merman_ros2_types__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__merman_ros2_types__msg__PoseArrivedData() -> *const std::ffi::c_void;
}

#[link(name = "merman_ros2_types__rosidl_generator_c")]
extern "C" {
    fn merman_ros2_types__msg__PoseArrivedData__init(msg: *mut PoseArrivedData) -> bool;
    fn merman_ros2_types__msg__PoseArrivedData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PoseArrivedData>, size: usize) -> bool;
    fn merman_ros2_types__msg__PoseArrivedData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PoseArrivedData>);
    fn merman_ros2_types__msg__PoseArrivedData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PoseArrivedData>, out_seq: *mut rosidl_runtime_rs::Sequence<PoseArrivedData>) -> bool;
}

// Corresponds to merman_ros2_types__msg__PoseArrivedData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseArrivedData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub pose_id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub play_audio: bool,

}



impl Default for PoseArrivedData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !merman_ros2_types__msg__PoseArrivedData__init(&mut msg as *mut _) {
        panic!("Call to merman_ros2_types__msg__PoseArrivedData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PoseArrivedData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__msg__PoseArrivedData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__msg__PoseArrivedData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { merman_ros2_types__msg__PoseArrivedData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PoseArrivedData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PoseArrivedData where Self: Sized {
  const TYPE_NAME: &'static str = "merman_ros2_types/msg/PoseArrivedData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__merman_ros2_types__msg__PoseArrivedData() }
  }
}


