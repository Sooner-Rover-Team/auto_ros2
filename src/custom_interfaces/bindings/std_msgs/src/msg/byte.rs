use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Byte {
    pub data: u8,
}

extern "C" {
    fn std_msgs__msg__Byte__init(msg: *mut Byte) -> bool;
    fn std_msgs__msg__Byte__fini(msg: *mut Byte);
    fn std_msgs__msg__Byte__are_equal(lhs: *const Byte, rhs: *const Byte) -> bool;
    fn std_msgs__msg__Byte__Sequence__init(msg: *mut ByteSeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Byte__Sequence__fini(msg: *mut ByteSeqRaw);
    fn std_msgs__msg__Byte__Sequence__are_equal(lhs: *const ByteSeqRaw, rhs: *const ByteSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Byte() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Byte {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Byte()
        }
    }
}

impl PartialEq for Byte {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Byte__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for ByteSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = ByteSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = ByteSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Byte__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Byte {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Byte__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Byte {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Byte__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct ByteSeqRaw {
    data: *mut Byte,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Byte.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct ByteSeq<const N: usize> {
    data: *mut Byte,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> ByteSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: ByteSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Byte__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: ByteSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Byte] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Byte] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Byte> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Byte> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for ByteSeq<N> {
    fn drop(&mut self) {
        let mut msg = ByteSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Byte__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for ByteSeq<N> {}
unsafe impl<const N: usize> Sync for ByteSeq<N> {}
