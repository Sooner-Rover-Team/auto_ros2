use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Bool {
    pub data: bool,
}

extern "C" {
    fn std_msgs__msg__Bool__init(msg: *mut Bool) -> bool;
    fn std_msgs__msg__Bool__fini(msg: *mut Bool);
    fn std_msgs__msg__Bool__are_equal(lhs: *const Bool, rhs: *const Bool) -> bool;
    fn std_msgs__msg__Bool__Sequence__init(msg: *mut BoolSeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Bool__Sequence__fini(msg: *mut BoolSeqRaw);
    fn std_msgs__msg__Bool__Sequence__are_equal(lhs: *const BoolSeqRaw, rhs: *const BoolSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Bool() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Bool {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Bool()
        }
    }
}

impl PartialEq for Bool {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Bool__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for BoolSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = BoolSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = BoolSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Bool__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Bool {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Bool__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Bool {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Bool__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct BoolSeqRaw {
    data: *mut Bool,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Bool.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct BoolSeq<const N: usize> {
    data: *mut Bool,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> BoolSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: BoolSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Bool__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: BoolSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Bool] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Bool] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Bool> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Bool> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for BoolSeq<N> {
    fn drop(&mut self) {
        let mut msg = BoolSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Bool__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for BoolSeq<N> {}
unsafe impl<const N: usize> Sync for BoolSeq<N> {}
