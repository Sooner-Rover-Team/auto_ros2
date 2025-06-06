use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct UInt16MultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: safe_drive::msg::U16Seq<0>,
}

extern "C" {
    fn std_msgs__msg__UInt16MultiArray__init(msg: *mut UInt16MultiArray) -> bool;
    fn std_msgs__msg__UInt16MultiArray__fini(msg: *mut UInt16MultiArray);
    fn std_msgs__msg__UInt16MultiArray__are_equal(lhs: *const UInt16MultiArray, rhs: *const UInt16MultiArray) -> bool;
    fn std_msgs__msg__UInt16MultiArray__Sequence__init(msg: *mut UInt16MultiArraySeqRaw, size: usize) -> bool;
    fn std_msgs__msg__UInt16MultiArray__Sequence__fini(msg: *mut UInt16MultiArraySeqRaw);
    fn std_msgs__msg__UInt16MultiArray__Sequence__are_equal(lhs: *const UInt16MultiArraySeqRaw, rhs: *const UInt16MultiArraySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt16MultiArray() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for UInt16MultiArray {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt16MultiArray()
        }
    }
}

impl PartialEq for UInt16MultiArray {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__UInt16MultiArray__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for UInt16MultiArraySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = UInt16MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = UInt16MultiArraySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__UInt16MultiArray__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl UInt16MultiArray {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__UInt16MultiArray__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for UInt16MultiArray {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__UInt16MultiArray__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct UInt16MultiArraySeqRaw {
    data: *mut UInt16MultiArray,
    size: size_t,
    capacity: size_t,
}

/// Sequence of UInt16MultiArray.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct UInt16MultiArraySeq<const N: usize> {
    data: *mut UInt16MultiArray,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> UInt16MultiArraySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: UInt16MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__UInt16MultiArray__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: UInt16MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[UInt16MultiArray] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [UInt16MultiArray] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, UInt16MultiArray> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, UInt16MultiArray> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for UInt16MultiArraySeq<N> {
    fn drop(&mut self) {
        let mut msg = UInt16MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__UInt16MultiArray__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for UInt16MultiArraySeq<N> {}
unsafe impl<const N: usize> Sync for UInt16MultiArraySeq<N> {}
