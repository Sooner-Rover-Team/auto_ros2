use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Vector3Stamped {
    pub header: std_msgs::msg::Header,
    pub vector: crate::msg::Vector3,
}

extern "C" {
    fn geometry_msgs__msg__Vector3Stamped__init(msg: *mut Vector3Stamped) -> bool;
    fn geometry_msgs__msg__Vector3Stamped__fini(msg: *mut Vector3Stamped);
    fn geometry_msgs__msg__Vector3Stamped__are_equal(lhs: *const Vector3Stamped, rhs: *const Vector3Stamped) -> bool;
    fn geometry_msgs__msg__Vector3Stamped__Sequence__init(msg: *mut Vector3StampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__Vector3Stamped__Sequence__fini(msg: *mut Vector3StampedSeqRaw);
    fn geometry_msgs__msg__Vector3Stamped__Sequence__are_equal(lhs: *const Vector3StampedSeqRaw, rhs: *const Vector3StampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Vector3Stamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Vector3Stamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Vector3Stamped()
        }
    }
}

impl PartialEq for Vector3Stamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__Vector3Stamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Vector3StampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Vector3StampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Vector3StampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__Vector3Stamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Vector3Stamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Vector3Stamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Vector3Stamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__Vector3Stamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Vector3StampedSeqRaw {
    data: *mut Vector3Stamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Vector3Stamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Vector3StampedSeq<const N: usize> {
    data: *mut Vector3Stamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Vector3StampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Vector3StampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Vector3Stamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Vector3StampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Vector3Stamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Vector3Stamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Vector3Stamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Vector3Stamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Vector3StampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = Vector3StampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__Vector3Stamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Vector3StampedSeq<N> {}
unsafe impl<const N: usize> Sync for Vector3StampedSeq<N> {}
