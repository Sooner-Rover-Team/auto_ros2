use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct ByteMultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: safe_drive::msg::U8Seq<0>,
}

extern "C" {
    fn std_msgs__msg__ByteMultiArray__init(msg: *mut ByteMultiArray) -> bool;
    fn std_msgs__msg__ByteMultiArray__fini(msg: *mut ByteMultiArray);
    fn std_msgs__msg__ByteMultiArray__are_equal(lhs: *const ByteMultiArray, rhs: *const ByteMultiArray) -> bool;
    fn std_msgs__msg__ByteMultiArray__Sequence__init(msg: *mut ByteMultiArraySeqRaw, size: usize) -> bool;
    fn std_msgs__msg__ByteMultiArray__Sequence__fini(msg: *mut ByteMultiArraySeqRaw);
    fn std_msgs__msg__ByteMultiArray__Sequence__are_equal(lhs: *const ByteMultiArraySeqRaw, rhs: *const ByteMultiArraySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__ByteMultiArray() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for ByteMultiArray {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__ByteMultiArray()
        }
    }
}

impl PartialEq for ByteMultiArray {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__ByteMultiArray__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for ByteMultiArraySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = ByteMultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = ByteMultiArraySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__ByteMultiArray__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl ByteMultiArray {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__ByteMultiArray__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for ByteMultiArray {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__ByteMultiArray__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct ByteMultiArraySeqRaw {
    data: *mut ByteMultiArray,
    size: size_t,
    capacity: size_t,
}

/// Sequence of ByteMultiArray.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct ByteMultiArraySeq<const N: usize> {
    data: *mut ByteMultiArray,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> ByteMultiArraySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: ByteMultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__ByteMultiArray__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: ByteMultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[ByteMultiArray] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [ByteMultiArray] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, ByteMultiArray> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, ByteMultiArray> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for ByteMultiArraySeq<N> {
    fn drop(&mut self) {
        let mut msg = ByteMultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__ByteMultiArray__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for ByteMultiArraySeq<N> {}
unsafe impl<const N: usize> Sync for ByteMultiArraySeq<N> {}
