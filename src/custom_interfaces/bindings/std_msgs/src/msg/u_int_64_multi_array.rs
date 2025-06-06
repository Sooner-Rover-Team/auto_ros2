use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct UInt64MultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: safe_drive::msg::U64Seq<0>,
}

extern "C" {
    fn std_msgs__msg__UInt64MultiArray__init(msg: *mut UInt64MultiArray) -> bool;
    fn std_msgs__msg__UInt64MultiArray__fini(msg: *mut UInt64MultiArray);
    fn std_msgs__msg__UInt64MultiArray__are_equal(lhs: *const UInt64MultiArray, rhs: *const UInt64MultiArray) -> bool;
    fn std_msgs__msg__UInt64MultiArray__Sequence__init(msg: *mut UInt64MultiArraySeqRaw, size: usize) -> bool;
    fn std_msgs__msg__UInt64MultiArray__Sequence__fini(msg: *mut UInt64MultiArraySeqRaw);
    fn std_msgs__msg__UInt64MultiArray__Sequence__are_equal(lhs: *const UInt64MultiArraySeqRaw, rhs: *const UInt64MultiArraySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt64MultiArray() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for UInt64MultiArray {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt64MultiArray()
        }
    }
}

impl PartialEq for UInt64MultiArray {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__UInt64MultiArray__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for UInt64MultiArraySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = UInt64MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = UInt64MultiArraySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__UInt64MultiArray__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl UInt64MultiArray {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__UInt64MultiArray__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for UInt64MultiArray {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__UInt64MultiArray__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct UInt64MultiArraySeqRaw {
    data: *mut UInt64MultiArray,
    size: size_t,
    capacity: size_t,
}

/// Sequence of UInt64MultiArray.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct UInt64MultiArraySeq<const N: usize> {
    data: *mut UInt64MultiArray,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> UInt64MultiArraySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: UInt64MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__UInt64MultiArray__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: UInt64MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[UInt64MultiArray] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [UInt64MultiArray] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, UInt64MultiArray> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, UInt64MultiArray> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for UInt64MultiArraySeq<N> {
    fn drop(&mut self) {
        let mut msg = UInt64MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__UInt64MultiArray__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for UInt64MultiArraySeq<N> {}
unsafe impl<const N: usize> Sync for UInt64MultiArraySeq<N> {}
