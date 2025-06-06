use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

extern "C" {
    fn geometry_msgs__msg__Quaternion__init(msg: *mut Quaternion) -> bool;
    fn geometry_msgs__msg__Quaternion__fini(msg: *mut Quaternion);
    fn geometry_msgs__msg__Quaternion__are_equal(lhs: *const Quaternion, rhs: *const Quaternion) -> bool;
    fn geometry_msgs__msg__Quaternion__Sequence__init(msg: *mut QuaternionSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__Quaternion__Sequence__fini(msg: *mut QuaternionSeqRaw);
    fn geometry_msgs__msg__Quaternion__Sequence__are_equal(lhs: *const QuaternionSeqRaw, rhs: *const QuaternionSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Quaternion() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Quaternion {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Quaternion()
        }
    }
}

impl PartialEq for Quaternion {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__Quaternion__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for QuaternionSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = QuaternionSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = QuaternionSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__Quaternion__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Quaternion {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Quaternion__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Quaternion {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__Quaternion__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct QuaternionSeqRaw {
    data: *mut Quaternion,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Quaternion.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct QuaternionSeq<const N: usize> {
    data: *mut Quaternion,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> QuaternionSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: QuaternionSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Quaternion__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: QuaternionSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Quaternion] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Quaternion] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Quaternion> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Quaternion> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for QuaternionSeq<N> {
    fn drop(&mut self) {
        let mut msg = QuaternionSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__Quaternion__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for QuaternionSeq<N> {}
unsafe impl<const N: usize> Sync for QuaternionSeq<N> {}
