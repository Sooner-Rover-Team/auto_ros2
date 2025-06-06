use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct TwistWithCovarianceStamped {
    pub header: std_msgs::msg::Header,
    pub twist: crate::msg::TwistWithCovariance,
}

extern "C" {
    fn geometry_msgs__msg__TwistWithCovarianceStamped__init(msg: *mut TwistWithCovarianceStamped) -> bool;
    fn geometry_msgs__msg__TwistWithCovarianceStamped__fini(msg: *mut TwistWithCovarianceStamped);
    fn geometry_msgs__msg__TwistWithCovarianceStamped__are_equal(lhs: *const TwistWithCovarianceStamped, rhs: *const TwistWithCovarianceStamped) -> bool;
    fn geometry_msgs__msg__TwistWithCovarianceStamped__Sequence__init(msg: *mut TwistWithCovarianceStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__TwistWithCovarianceStamped__Sequence__fini(msg: *mut TwistWithCovarianceStampedSeqRaw);
    fn geometry_msgs__msg__TwistWithCovarianceStamped__Sequence__are_equal(lhs: *const TwistWithCovarianceStampedSeqRaw, rhs: *const TwistWithCovarianceStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TwistWithCovarianceStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TwistWithCovarianceStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TwistWithCovarianceStamped()
        }
    }
}

impl PartialEq for TwistWithCovarianceStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__TwistWithCovarianceStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TwistWithCovarianceStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TwistWithCovarianceStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TwistWithCovarianceStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__TwistWithCovarianceStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TwistWithCovarianceStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__TwistWithCovarianceStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TwistWithCovarianceStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__TwistWithCovarianceStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TwistWithCovarianceStampedSeqRaw {
    data: *mut TwistWithCovarianceStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TwistWithCovarianceStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TwistWithCovarianceStampedSeq<const N: usize> {
    data: *mut TwistWithCovarianceStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TwistWithCovarianceStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TwistWithCovarianceStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__TwistWithCovarianceStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TwistWithCovarianceStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TwistWithCovarianceStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TwistWithCovarianceStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TwistWithCovarianceStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TwistWithCovarianceStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TwistWithCovarianceStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = TwistWithCovarianceStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__TwistWithCovarianceStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TwistWithCovarianceStampedSeq<N> {}
unsafe impl<const N: usize> Sync for TwistWithCovarianceStampedSeq<N> {}
