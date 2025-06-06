use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Char {
    pub data: u8,
}

extern "C" {
    fn std_msgs__msg__Char__init(msg: *mut Char) -> bool;
    fn std_msgs__msg__Char__fini(msg: *mut Char);
    fn std_msgs__msg__Char__are_equal(lhs: *const Char, rhs: *const Char) -> bool;
    fn std_msgs__msg__Char__Sequence__init(msg: *mut CharSeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Char__Sequence__fini(msg: *mut CharSeqRaw);
    fn std_msgs__msg__Char__Sequence__are_equal(lhs: *const CharSeqRaw, rhs: *const CharSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Char() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Char {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Char()
        }
    }
}

impl PartialEq for Char {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Char__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for CharSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = CharSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = CharSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Char__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Char {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Char__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Char {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Char__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct CharSeqRaw {
    data: *mut Char,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Char.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct CharSeq<const N: usize> {
    data: *mut Char,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> CharSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: CharSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Char__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: CharSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Char] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Char] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Char> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Char> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for CharSeq<N> {
    fn drop(&mut self) {
        let mut msg = CharSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Char__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for CharSeq<N> {}
unsafe impl<const N: usize> Sync for CharSeq<N> {}
