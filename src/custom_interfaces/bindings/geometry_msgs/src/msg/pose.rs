use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Pose {
    pub position: crate::msg::Point,
    pub orientation: crate::msg::Quaternion,
}

extern "C" {
    fn geometry_msgs__msg__Pose__init(msg: *mut Pose) -> bool;
    fn geometry_msgs__msg__Pose__fini(msg: *mut Pose);
    fn geometry_msgs__msg__Pose__are_equal(lhs: *const Pose, rhs: *const Pose) -> bool;
    fn geometry_msgs__msg__Pose__Sequence__init(msg: *mut PoseSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__Pose__Sequence__fini(msg: *mut PoseSeqRaw);
    fn geometry_msgs__msg__Pose__Sequence__are_equal(lhs: *const PoseSeqRaw, rhs: *const PoseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Pose() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Pose {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Pose()
        }
    }
}

impl PartialEq for Pose {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__Pose__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for PoseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = PoseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = PoseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__Pose__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Pose {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Pose__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Pose {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__Pose__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct PoseSeqRaw {
    data: *mut Pose,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Pose.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct PoseSeq<const N: usize> {
    data: *mut Pose,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> PoseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: PoseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Pose__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: PoseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Pose] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Pose] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Pose> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Pose> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for PoseSeq<N> {
    fn drop(&mut self) {
        let mut msg = PoseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__Pose__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for PoseSeq<N> {}
unsafe impl<const N: usize> Sync for PoseSeq<N> {}
