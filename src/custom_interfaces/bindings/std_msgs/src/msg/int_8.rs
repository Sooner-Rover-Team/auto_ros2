use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Int8 {
    pub data: i8,
}

extern "C" {
    fn std_msgs__msg__Int8__init(msg: *mut Int8) -> bool;
    fn std_msgs__msg__Int8__fini(msg: *mut Int8);
    fn std_msgs__msg__Int8__are_equal(lhs: *const Int8, rhs: *const Int8) -> bool;
    fn std_msgs__msg__Int8__Sequence__init(msg: *mut Int8SeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Int8__Sequence__fini(msg: *mut Int8SeqRaw);
    fn std_msgs__msg__Int8__Sequence__are_equal(lhs: *const Int8SeqRaw, rhs: *const Int8SeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int8() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Int8 {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int8()
        }
    }
}

impl PartialEq for Int8 {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Int8__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Int8Seq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Int8SeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Int8SeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Int8__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Int8 {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Int8__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Int8 {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Int8__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Int8SeqRaw {
    data: *mut Int8,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Int8.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Int8Seq<const N: usize> {
    data: *mut Int8,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Int8Seq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Int8SeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Int8__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Int8SeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Int8] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Int8] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Int8> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Int8> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Int8Seq<N> {
    fn drop(&mut self) {
        let mut msg = Int8SeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Int8__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Int8Seq<N> {}
unsafe impl<const N: usize> Sync for Int8Seq<N> {}
