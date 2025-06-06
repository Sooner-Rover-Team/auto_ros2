use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct MultiArrayLayout {
    pub dim: crate::msg::MultiArrayDimensionSeq<0>,
    pub data_offset: u32,
}

extern "C" {
    fn std_msgs__msg__MultiArrayLayout__init(msg: *mut MultiArrayLayout) -> bool;
    fn std_msgs__msg__MultiArrayLayout__fini(msg: *mut MultiArrayLayout);
    fn std_msgs__msg__MultiArrayLayout__are_equal(lhs: *const MultiArrayLayout, rhs: *const MultiArrayLayout) -> bool;
    fn std_msgs__msg__MultiArrayLayout__Sequence__init(msg: *mut MultiArrayLayoutSeqRaw, size: usize) -> bool;
    fn std_msgs__msg__MultiArrayLayout__Sequence__fini(msg: *mut MultiArrayLayoutSeqRaw);
    fn std_msgs__msg__MultiArrayLayout__Sequence__are_equal(lhs: *const MultiArrayLayoutSeqRaw, rhs: *const MultiArrayLayoutSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__MultiArrayLayout() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for MultiArrayLayout {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__MultiArrayLayout()
        }
    }
}

impl PartialEq for MultiArrayLayout {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__MultiArrayLayout__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for MultiArrayLayoutSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = MultiArrayLayoutSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = MultiArrayLayoutSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__MultiArrayLayout__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl MultiArrayLayout {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__MultiArrayLayout__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for MultiArrayLayout {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__MultiArrayLayout__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct MultiArrayLayoutSeqRaw {
    data: *mut MultiArrayLayout,
    size: size_t,
    capacity: size_t,
}

/// Sequence of MultiArrayLayout.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct MultiArrayLayoutSeq<const N: usize> {
    data: *mut MultiArrayLayout,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> MultiArrayLayoutSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: MultiArrayLayoutSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__MultiArrayLayout__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: MultiArrayLayoutSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[MultiArrayLayout] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [MultiArrayLayout] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, MultiArrayLayout> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, MultiArrayLayout> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for MultiArrayLayoutSeq<N> {
    fn drop(&mut self) {
        let mut msg = MultiArrayLayoutSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__MultiArrayLayout__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for MultiArrayLayoutSeq<N> {}
unsafe impl<const N: usize> Sync for MultiArrayLayoutSeq<N> {}
