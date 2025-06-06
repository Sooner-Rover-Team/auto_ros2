use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct MultiArrayDimension {
    pub label: safe_drive::msg::RosString<0>,
    pub size: u32,
    pub stride: u32,
}

extern "C" {
    fn std_msgs__msg__MultiArrayDimension__init(msg: *mut MultiArrayDimension) -> bool;
    fn std_msgs__msg__MultiArrayDimension__fini(msg: *mut MultiArrayDimension);
    fn std_msgs__msg__MultiArrayDimension__are_equal(lhs: *const MultiArrayDimension, rhs: *const MultiArrayDimension) -> bool;
    fn std_msgs__msg__MultiArrayDimension__Sequence__init(msg: *mut MultiArrayDimensionSeqRaw, size: usize) -> bool;
    fn std_msgs__msg__MultiArrayDimension__Sequence__fini(msg: *mut MultiArrayDimensionSeqRaw);
    fn std_msgs__msg__MultiArrayDimension__Sequence__are_equal(lhs: *const MultiArrayDimensionSeqRaw, rhs: *const MultiArrayDimensionSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__MultiArrayDimension() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for MultiArrayDimension {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__MultiArrayDimension()
        }
    }
}

impl PartialEq for MultiArrayDimension {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__MultiArrayDimension__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for MultiArrayDimensionSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = MultiArrayDimensionSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = MultiArrayDimensionSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__MultiArrayDimension__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl MultiArrayDimension {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__MultiArrayDimension__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for MultiArrayDimension {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__MultiArrayDimension__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct MultiArrayDimensionSeqRaw {
    data: *mut MultiArrayDimension,
    size: size_t,
    capacity: size_t,
}

/// Sequence of MultiArrayDimension.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct MultiArrayDimensionSeq<const N: usize> {
    data: *mut MultiArrayDimension,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> MultiArrayDimensionSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: MultiArrayDimensionSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__MultiArrayDimension__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: MultiArrayDimensionSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[MultiArrayDimension] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [MultiArrayDimension] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, MultiArrayDimension> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, MultiArrayDimension> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for MultiArrayDimensionSeq<N> {
    fn drop(&mut self) {
        let mut msg = MultiArrayDimensionSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__MultiArrayDimension__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for MultiArrayDimensionSeq<N> {}
unsafe impl<const N: usize> Sync for MultiArrayDimensionSeq<N> {}
