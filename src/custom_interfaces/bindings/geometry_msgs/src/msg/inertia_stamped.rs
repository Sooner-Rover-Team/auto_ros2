use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct InertiaStamped {
    pub header: std_msgs::msg::Header,
    pub inertia: crate::msg::Inertia,
}

extern "C" {
    fn geometry_msgs__msg__InertiaStamped__init(msg: *mut InertiaStamped) -> bool;
    fn geometry_msgs__msg__InertiaStamped__fini(msg: *mut InertiaStamped);
    fn geometry_msgs__msg__InertiaStamped__are_equal(lhs: *const InertiaStamped, rhs: *const InertiaStamped) -> bool;
    fn geometry_msgs__msg__InertiaStamped__Sequence__init(msg: *mut InertiaStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__InertiaStamped__Sequence__fini(msg: *mut InertiaStampedSeqRaw);
    fn geometry_msgs__msg__InertiaStamped__Sequence__are_equal(lhs: *const InertiaStampedSeqRaw, rhs: *const InertiaStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__InertiaStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for InertiaStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__InertiaStamped()
        }
    }
}

impl PartialEq for InertiaStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__InertiaStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for InertiaStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = InertiaStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = InertiaStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__InertiaStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl InertiaStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__InertiaStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for InertiaStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__InertiaStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct InertiaStampedSeqRaw {
    data: *mut InertiaStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of InertiaStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct InertiaStampedSeq<const N: usize> {
    data: *mut InertiaStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> InertiaStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: InertiaStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__InertiaStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: InertiaStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[InertiaStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [InertiaStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, InertiaStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, InertiaStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for InertiaStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = InertiaStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__InertiaStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for InertiaStampedSeq<N> {}
unsafe impl<const N: usize> Sync for InertiaStampedSeq<N> {}
