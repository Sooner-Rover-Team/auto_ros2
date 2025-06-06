use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct VelocityStamped {
    pub header: std_msgs::msg::Header,
    pub body_frame_id: safe_drive::msg::RosString<0>,
    pub reference_frame_id: safe_drive::msg::RosString<0>,
    pub velocity: crate::msg::Twist,
}

extern "C" {
    fn geometry_msgs__msg__VelocityStamped__init(msg: *mut VelocityStamped) -> bool;
    fn geometry_msgs__msg__VelocityStamped__fini(msg: *mut VelocityStamped);
    fn geometry_msgs__msg__VelocityStamped__are_equal(lhs: *const VelocityStamped, rhs: *const VelocityStamped) -> bool;
    fn geometry_msgs__msg__VelocityStamped__Sequence__init(msg: *mut VelocityStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__VelocityStamped__Sequence__fini(msg: *mut VelocityStampedSeqRaw);
    fn geometry_msgs__msg__VelocityStamped__Sequence__are_equal(lhs: *const VelocityStampedSeqRaw, rhs: *const VelocityStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__VelocityStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for VelocityStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__VelocityStamped()
        }
    }
}

impl PartialEq for VelocityStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__VelocityStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for VelocityStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = VelocityStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = VelocityStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__VelocityStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl VelocityStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__VelocityStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for VelocityStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__VelocityStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct VelocityStampedSeqRaw {
    data: *mut VelocityStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of VelocityStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct VelocityStampedSeq<const N: usize> {
    data: *mut VelocityStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> VelocityStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: VelocityStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__VelocityStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: VelocityStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[VelocityStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [VelocityStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, VelocityStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, VelocityStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for VelocityStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = VelocityStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__VelocityStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for VelocityStampedSeq<N> {}
unsafe impl<const N: usize> Sync for VelocityStampedSeq<N> {}
