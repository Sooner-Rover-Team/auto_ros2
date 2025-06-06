use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Twist {
    pub linear: crate::msg::Vector3,
    pub angular: crate::msg::Vector3,
}

extern "C" {
    fn geometry_msgs__msg__Twist__init(msg: *mut Twist) -> bool;
    fn geometry_msgs__msg__Twist__fini(msg: *mut Twist);
    fn geometry_msgs__msg__Twist__are_equal(lhs: *const Twist, rhs: *const Twist) -> bool;
    fn geometry_msgs__msg__Twist__Sequence__init(msg: *mut TwistSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__Twist__Sequence__fini(msg: *mut TwistSeqRaw);
    fn geometry_msgs__msg__Twist__Sequence__are_equal(lhs: *const TwistSeqRaw, rhs: *const TwistSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Twist() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Twist {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Twist()
        }
    }
}

impl PartialEq for Twist {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__Twist__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TwistSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TwistSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TwistSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__Twist__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Twist {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Twist__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Twist {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__Twist__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TwistSeqRaw {
    data: *mut Twist,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Twist.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TwistSeq<const N: usize> {
    data: *mut Twist,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TwistSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TwistSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Twist__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TwistSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Twist] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Twist] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Twist> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Twist> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TwistSeq<N> {
    fn drop(&mut self) {
        let mut msg = TwistSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__Twist__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TwistSeq<N> {}
unsafe impl<const N: usize> Sync for TwistSeq<N> {}
