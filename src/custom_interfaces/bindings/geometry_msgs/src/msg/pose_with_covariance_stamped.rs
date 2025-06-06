use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct PoseWithCovarianceStamped {
    pub header: std_msgs::msg::Header,
    pub pose: crate::msg::PoseWithCovariance,
}

extern "C" {
    fn geometry_msgs__msg__PoseWithCovarianceStamped__init(msg: *mut PoseWithCovarianceStamped) -> bool;
    fn geometry_msgs__msg__PoseWithCovarianceStamped__fini(msg: *mut PoseWithCovarianceStamped);
    fn geometry_msgs__msg__PoseWithCovarianceStamped__are_equal(lhs: *const PoseWithCovarianceStamped, rhs: *const PoseWithCovarianceStamped) -> bool;
    fn geometry_msgs__msg__PoseWithCovarianceStamped__Sequence__init(msg: *mut PoseWithCovarianceStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__PoseWithCovarianceStamped__Sequence__fini(msg: *mut PoseWithCovarianceStampedSeqRaw);
    fn geometry_msgs__msg__PoseWithCovarianceStamped__Sequence__are_equal(lhs: *const PoseWithCovarianceStampedSeqRaw, rhs: *const PoseWithCovarianceStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PoseWithCovarianceStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for PoseWithCovarianceStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PoseWithCovarianceStamped()
        }
    }
}

impl PartialEq for PoseWithCovarianceStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__PoseWithCovarianceStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for PoseWithCovarianceStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = PoseWithCovarianceStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = PoseWithCovarianceStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__PoseWithCovarianceStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl PoseWithCovarianceStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__PoseWithCovarianceStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for PoseWithCovarianceStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__PoseWithCovarianceStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct PoseWithCovarianceStampedSeqRaw {
    data: *mut PoseWithCovarianceStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of PoseWithCovarianceStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct PoseWithCovarianceStampedSeq<const N: usize> {
    data: *mut PoseWithCovarianceStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> PoseWithCovarianceStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: PoseWithCovarianceStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__PoseWithCovarianceStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: PoseWithCovarianceStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[PoseWithCovarianceStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [PoseWithCovarianceStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, PoseWithCovarianceStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, PoseWithCovarianceStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for PoseWithCovarianceStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = PoseWithCovarianceStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__PoseWithCovarianceStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for PoseWithCovarianceStampedSeq<N> {}
unsafe impl<const N: usize> Sync for PoseWithCovarianceStampedSeq<N> {}
