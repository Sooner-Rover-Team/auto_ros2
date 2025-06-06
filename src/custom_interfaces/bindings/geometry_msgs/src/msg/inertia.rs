use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Inertia {
    pub m: f64,
    pub com: crate::msg::Vector3,
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

extern "C" {
    fn geometry_msgs__msg__Inertia__init(msg: *mut Inertia) -> bool;
    fn geometry_msgs__msg__Inertia__fini(msg: *mut Inertia);
    fn geometry_msgs__msg__Inertia__are_equal(lhs: *const Inertia, rhs: *const Inertia) -> bool;
    fn geometry_msgs__msg__Inertia__Sequence__init(msg: *mut InertiaSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__Inertia__Sequence__fini(msg: *mut InertiaSeqRaw);
    fn geometry_msgs__msg__Inertia__Sequence__are_equal(lhs: *const InertiaSeqRaw, rhs: *const InertiaSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Inertia() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Inertia {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__Inertia()
        }
    }
}

impl PartialEq for Inertia {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__Inertia__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for InertiaSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = InertiaSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = InertiaSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__Inertia__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Inertia {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Inertia__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Inertia {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__Inertia__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct InertiaSeqRaw {
    data: *mut Inertia,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Inertia.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct InertiaSeq<const N: usize> {
    data: *mut Inertia,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> InertiaSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: InertiaSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__Inertia__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: InertiaSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Inertia] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Inertia] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Inertia> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Inertia> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for InertiaSeq<N> {
    fn drop(&mut self) {
        let mut msg = InertiaSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__Inertia__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for InertiaSeq<N> {}
unsafe impl<const N: usize> Sync for InertiaSeq<N> {}
