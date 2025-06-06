use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Int16 {
    pub data: i16,
}

extern "C" {
    fn std_msgs__msg__Int16__init(msg: *mut Int16) -> bool;
    fn std_msgs__msg__Int16__fini(msg: *mut Int16);
    fn std_msgs__msg__Int16__are_equal(lhs: *const Int16, rhs: *const Int16) -> bool;
    fn std_msgs__msg__Int16__Sequence__init(msg: *mut Int16SeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Int16__Sequence__fini(msg: *mut Int16SeqRaw);
    fn std_msgs__msg__Int16__Sequence__are_equal(lhs: *const Int16SeqRaw, rhs: *const Int16SeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int16() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Int16 {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int16()
        }
    }
}

impl PartialEq for Int16 {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Int16__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Int16Seq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Int16SeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Int16SeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Int16__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Int16 {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Int16__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Int16 {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Int16__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Int16SeqRaw {
    data: *mut Int16,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Int16.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Int16Seq<const N: usize> {
    data: *mut Int16,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Int16Seq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Int16SeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Int16__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Int16SeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Int16] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Int16] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Int16> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Int16> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Int16Seq<N> {
    fn drop(&mut self) {
        let mut msg = Int16SeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Int16__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Int16Seq<N> {}
unsafe impl<const N: usize> Sync for Int16Seq<N> {}
