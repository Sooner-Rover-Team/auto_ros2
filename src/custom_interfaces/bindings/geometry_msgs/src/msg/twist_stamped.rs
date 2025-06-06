use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct TwistStamped {
    pub header: std_msgs::msg::Header,
    pub twist: crate::msg::Twist,
}

extern "C" {
    fn geometry_msgs__msg__TwistStamped__init(msg: *mut TwistStamped) -> bool;
    fn geometry_msgs__msg__TwistStamped__fini(msg: *mut TwistStamped);
    fn geometry_msgs__msg__TwistStamped__are_equal(lhs: *const TwistStamped, rhs: *const TwistStamped) -> bool;
    fn geometry_msgs__msg__TwistStamped__Sequence__init(msg: *mut TwistStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__TwistStamped__Sequence__fini(msg: *mut TwistStampedSeqRaw);
    fn geometry_msgs__msg__TwistStamped__Sequence__are_equal(lhs: *const TwistStampedSeqRaw, rhs: *const TwistStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TwistStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TwistStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TwistStamped()
        }
    }
}

impl PartialEq for TwistStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__TwistStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TwistStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TwistStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TwistStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__TwistStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TwistStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__TwistStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TwistStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__TwistStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TwistStampedSeqRaw {
    data: *mut TwistStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TwistStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TwistStampedSeq<const N: usize> {
    data: *mut TwistStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TwistStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TwistStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__TwistStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TwistStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TwistStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TwistStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TwistStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TwistStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TwistStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = TwistStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__TwistStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TwistStampedSeq<N> {}
unsafe impl<const N: usize> Sync for TwistStampedSeq<N> {}
