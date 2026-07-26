#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__msgs__msg__TargetDepth() -> *const std::ffi::c_void;
}

#[link(name = "msgs__rosidl_generator_c")]
extern "C" {
    fn msgs__msg__TargetDepth__init(msg: *mut TargetDepth) -> bool;
    fn msgs__msg__TargetDepth__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TargetDepth>, size: usize) -> bool;
    fn msgs__msg__TargetDepth__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TargetDepth>);
    fn msgs__msg__TargetDepth__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TargetDepth>, out_seq: *mut rosidl_runtime_rs::Sequence<TargetDepth>) -> bool;
}

// Corresponds to msgs__msg__TargetDepth
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TargetDepth {

    // This member is not documented.
    #[allow(missing_docs)]
    pub depth: f64,

}



impl Default for TargetDepth {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !msgs__msg__TargetDepth__init(&mut msg as *mut _) {
        panic!("Call to msgs__msg__TargetDepth__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TargetDepth {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msgs__msg__TargetDepth__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msgs__msg__TargetDepth__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { msgs__msg__TargetDepth__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TargetDepth {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TargetDepth where Self: Sized {
  const TYPE_NAME: &'static str = "msgs/msg/TargetDepth";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__msgs__msg__TargetDepth() }
  }
}


