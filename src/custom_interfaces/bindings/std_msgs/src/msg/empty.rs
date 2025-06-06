use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Empty {
    pub structure_needs_at_least_one_member: u8,
}

extern "C" {
    fn std_msgs__msg__Empty__init(msg: *mut Empty) -> bool;
    fn std_msgs__msg__Empty__fini(msg: *mut Empty);
    fn std_msgs__msg__Empty__are_equal(lhs: *const Empty, rhs: *const Empty) -> bool;
    fn std_msgs__msg__Empty__Sequence__init(msg: *mut EmptySeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Empty__Sequence__fini(msg: *mut EmptySeqRaw);
    fn std_msgs__msg__Empty__Sequence__are_equal(lhs: *const EmptySeqRaw, rhs: *const EmptySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Empty() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Empty {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Empty()
        }
    }
}

impl PartialEq for Empty {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Empty__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for EmptySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = EmptySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = EmptySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Empty__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Empty {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Empty__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Empty {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Empty__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct EmptySeqRaw {
    data: *mut Empty,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Empty.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct EmptySeq<const N: usize> {
    data: *mut Empty,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> EmptySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: EmptySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Empty__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: EmptySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Empty] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Empty] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Empty> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Empty> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for EmptySeq<N> {
    fn drop(&mut self) {
        let mut msg = EmptySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Empty__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for EmptySeq<N> {}
unsafe impl<const N: usize> Sync for EmptySeq<N> {}
