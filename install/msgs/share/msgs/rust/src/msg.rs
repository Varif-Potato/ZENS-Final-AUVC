#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to msgs__msg__TargetDepth

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TargetDepth {

    // This member is not documented.
    #[allow(missing_docs)]
    pub depth: f64,

}



impl Default for TargetDepth {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TargetDepth::default())
  }
}

impl rosidl_runtime_rs::Message for TargetDepth {
  type RmwMsg = super::msg::rmw::TargetDepth;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        depth: msg.depth,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      depth: msg.depth,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      depth: msg.depth,
    }
  }
}


