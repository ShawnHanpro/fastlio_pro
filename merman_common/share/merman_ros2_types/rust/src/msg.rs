#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to merman_ros2_types__msg__PoseArrivedData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PoseArrivedData::default())
  }
}

impl rosidl_runtime_rs::Message for PoseArrivedData {
  type RmwMsg = super::msg::rmw::PoseArrivedData;

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


