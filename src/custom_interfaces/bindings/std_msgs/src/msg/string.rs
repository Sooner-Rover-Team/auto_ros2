use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct String {
    pub data: safe_drive::msg::RosString<0>,
}

extern "C" {
    fn std_msgs__msg__String__init(msg: *mut String) -> bool;
    fn std_msgs__msg__String__fini(msg: *mut String);
    fn std_msgs__msg__String__are_equal(lhs: *const String, rhs: *const String) -> bool;
    fn std_msgs__msg__String__Sequence__init(msg: *mut StringSeqRaw, size: usize) -> bool;
    fn std_msgs__msg__String__Sequence__fini(msg: *mut StringSeqRaw);
    fn std_msgs__msg__String__Sequence__are_equal(lhs: *const StringSeqRaw, rhs: *const StringSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for String {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String()
        }
    }
}

impl PartialEq for String {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__String__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for StringSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = StringSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = StringSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__String__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl String {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__String__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for String {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__String__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct StringSeqRaw {
    data: *mut String,
    size: size_t,
    capacity: size_t,
}

/// Sequence of String.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct StringSeq<const N: usize> {
    data: *mut String,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> StringSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: StringSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__String__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: StringSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[String] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [String] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, String> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, String> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for StringSeq<N> {
    fn drop(&mut self) {
        let mut msg = StringSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__String__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for StringSeq<N> {}
unsafe impl<const N: usize> Sync for StringSeq<N> {}
