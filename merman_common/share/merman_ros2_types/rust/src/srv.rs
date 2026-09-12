#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to merman_ros2_types__srv__PoseArrived_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::PoseArrived_Request::default())
  }
}

impl rosidl_runtime_rs::Message for PoseArrived_Request {
  type RmwMsg = super::srv::rmw::PoseArrived_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pose_id: msg.pose_id,
        play_audio: msg.play_audio,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      pose_id: msg.pose_id,
      play_audio: msg.play_audio,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pose_id: msg.pose_id,
      play_audio: msg.play_audio,
    }
  }
}


// Corresponds to merman_ros2_types__srv__PoseArrived_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PoseArrived_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for PoseArrived_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::PoseArrived_Response::default())
  }
}

impl rosidl_runtime_rs::Message for PoseArrived_Response {
  type RmwMsg = super::srv::rmw::PoseArrived_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
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


