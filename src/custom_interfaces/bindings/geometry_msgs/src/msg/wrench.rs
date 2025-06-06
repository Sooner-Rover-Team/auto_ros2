use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Wrench {
    pub force: crate::msg::Vector3,
    pub torque: crate::msg::Vector3,
}

extern "C" {
    fn geometry_msgs__msg__Wrench__init(msg: *mut Wrench) -> bool;
    fn geometry_msgs__msg__Wrench__fini(msg: *mut Wrench);
    fn geometry_msgs__msg__Wrench__are_equal(lhs: *const Wrench, rhs: *const Wrench) -> bool;
    fn geometry_msgs__msg__Wrench__Sequence__init(msg: *mut WrenchSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__Wrench__Sequence__fini(msg: *mut WrenchSeqRaw);
    fn geometry_msgs__msg__Wrench__Sequence__are_equal(lhs: *const WrenchSeqRaw, rhs: *const WrenchSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Wrench() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Wrench {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Wrench()
        }
    }
}

impl PartialEq for Wrench {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__Wrench__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for WrenchSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = WrenchSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = WrenchSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__Wrench__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Wrench {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Wrench__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Wrench {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__Wrench__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct WrenchSeqRaw {
    data: *mut Wrench,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Wrench.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct WrenchSeq<const N: usize> {
    data: *mut Wrench,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> WrenchSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: WrenchSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Wrench__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: WrenchSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Wrench] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Wrench] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Wrench> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Wrench> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for WrenchSeq<N> {
    fn drop(&mut self) {
        let mut msg = WrenchSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__Wrench__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for WrenchSeq<N> {}
unsafe impl<const N: usize> Sync for WrenchSeq<N> {}
