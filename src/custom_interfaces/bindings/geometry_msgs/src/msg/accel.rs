use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Accel {
    pub linear: crate::msg::Vector3,
    pub angular: crate::msg::Vector3,
}

extern "C" {
    fn geometry_msgs__msg__Accel__init(msg: *mut Accel) -> bool;
    fn geometry_msgs__msg__Accel__fini(msg: *mut Accel);
    fn geometry_msgs__msg__Accel__are_equal(lhs: *const Accel, rhs: *const Accel) -> bool;
    fn geometry_msgs__msg__Accel__Sequence__init(msg: *mut AccelSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__Accel__Sequence__fini(msg: *mut AccelSeqRaw);
    fn geometry_msgs__msg__Accel__Sequence__are_equal(lhs: *const AccelSeqRaw, rhs: *const AccelSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Accel() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Accel {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Accel()
        }
    }
}

impl PartialEq for Accel {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__Accel__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for AccelSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = AccelSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = AccelSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__Accel__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Accel {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Accel__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Accel {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__Accel__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct AccelSeqRaw {
    data: *mut Accel,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Accel.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct AccelSeq<const N: usize> {
    data: *mut Accel,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> AccelSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: AccelSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Accel__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: AccelSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Accel] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Accel] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Accel> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Accel> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for AccelSeq<N> {
    fn drop(&mut self) {
        let mut msg = AccelSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__Accel__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for AccelSeq<N> {}
unsafe impl<const N: usize> Sync for AccelSeq<N> {}
