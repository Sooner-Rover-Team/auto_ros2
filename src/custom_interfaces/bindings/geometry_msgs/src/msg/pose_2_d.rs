use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

extern "C" {
    fn geometry_msgs__msg__Pose2D__init(msg: *mut Pose2D) -> bool;
    fn geometry_msgs__msg__Pose2D__fini(msg: *mut Pose2D);
    fn geometry_msgs__msg__Pose2D__are_equal(lhs: *const Pose2D, rhs: *const Pose2D) -> bool;
    fn geometry_msgs__msg__Pose2D__Sequence__init(msg: *mut Pose2DSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__Pose2D__Sequence__fini(msg: *mut Pose2DSeqRaw);
    fn geometry_msgs__msg__Pose2D__Sequence__are_equal(lhs: *const Pose2DSeqRaw, rhs: *const Pose2DSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Pose2D() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Pose2D {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Pose2D()
        }
    }
}

impl PartialEq for Pose2D {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__Pose2D__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Pose2DSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Pose2DSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Pose2DSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__Pose2D__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Pose2D {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Pose2D__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Pose2D {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__Pose2D__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Pose2DSeqRaw {
    data: *mut Pose2D,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Pose2D.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Pose2DSeq<const N: usize> {
    data: *mut Pose2D,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Pose2DSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Pose2DSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Pose2D__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Pose2DSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Pose2D] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Pose2D] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Pose2D> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Pose2D> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Pose2DSeq<N> {
    fn drop(&mut self) {
        let mut msg = Pose2DSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__Pose2D__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Pose2DSeq<N> {}
unsafe impl<const N: usize> Sync for Pose2DSeq<N> {}
