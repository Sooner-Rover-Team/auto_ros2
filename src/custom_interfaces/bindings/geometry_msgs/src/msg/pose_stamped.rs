use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct PoseStamped {
    pub header: std_msgs::msg::Header,
    pub pose: crate::msg::Pose,
}

extern "C" {
    fn geometry_msgs__msg__PoseStamped__init(msg: *mut PoseStamped) -> bool;
    fn geometry_msgs__msg__PoseStamped__fini(msg: *mut PoseStamped);
    fn geometry_msgs__msg__PoseStamped__are_equal(lhs: *const PoseStamped, rhs: *const PoseStamped) -> bool;
    fn geometry_msgs__msg__PoseStamped__Sequence__init(msg: *mut PoseStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__PoseStamped__Sequence__fini(msg: *mut PoseStampedSeqRaw);
    fn geometry_msgs__msg__PoseStamped__Sequence__are_equal(lhs: *const PoseStampedSeqRaw, rhs: *const PoseStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PoseStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for PoseStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PoseStamped()
        }
    }
}

impl PartialEq for PoseStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__PoseStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for PoseStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = PoseStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = PoseStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__PoseStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl PoseStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__PoseStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for PoseStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__PoseStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct PoseStampedSeqRaw {
    data: *mut PoseStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of PoseStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct PoseStampedSeq<const N: usize> {
    data: *mut PoseStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> PoseStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: PoseStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__PoseStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: PoseStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[PoseStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [PoseStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, PoseStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, PoseStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for PoseStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = PoseStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__PoseStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for PoseStampedSeq<N> {}
unsafe impl<const N: usize> Sync for PoseStampedSeq<N> {}
