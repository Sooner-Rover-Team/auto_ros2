use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct AccelWithCovarianceStamped {
    pub header: std_msgs::msg::Header,
    pub accel: crate::msg::AccelWithCovariance,
}

extern "C" {
    fn geometry_msgs__msg__AccelWithCovarianceStamped__init(msg: *mut AccelWithCovarianceStamped) -> bool;
    fn geometry_msgs__msg__AccelWithCovarianceStamped__fini(msg: *mut AccelWithCovarianceStamped);
    fn geometry_msgs__msg__AccelWithCovarianceStamped__are_equal(lhs: *const AccelWithCovarianceStamped, rhs: *const AccelWithCovarianceStamped) -> bool;
    fn geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__init(msg: *mut AccelWithCovarianceStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__fini(msg: *mut AccelWithCovarianceStampedSeqRaw);
    fn geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__are_equal(lhs: *const AccelWithCovarianceStampedSeqRaw, rhs: *const AccelWithCovarianceStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__AccelWithCovarianceStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for AccelWithCovarianceStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__AccelWithCovarianceStamped()
        }
    }
}

impl PartialEq for AccelWithCovarianceStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__AccelWithCovarianceStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for AccelWithCovarianceStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = AccelWithCovarianceStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = AccelWithCovarianceStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl AccelWithCovarianceStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__AccelWithCovarianceStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for AccelWithCovarianceStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__AccelWithCovarianceStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct AccelWithCovarianceStampedSeqRaw {
    data: *mut AccelWithCovarianceStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of AccelWithCovarianceStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct AccelWithCovarianceStampedSeq<const N: usize> {
    data: *mut AccelWithCovarianceStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> AccelWithCovarianceStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: AccelWithCovarianceStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: AccelWithCovarianceStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[AccelWithCovarianceStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [AccelWithCovarianceStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, AccelWithCovarianceStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, AccelWithCovarianceStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for AccelWithCovarianceStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = AccelWithCovarianceStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__AccelWithCovarianceStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for AccelWithCovarianceStampedSeq<N> {}
unsafe impl<const N: usize> Sync for AccelWithCovarianceStampedSeq<N> {}
