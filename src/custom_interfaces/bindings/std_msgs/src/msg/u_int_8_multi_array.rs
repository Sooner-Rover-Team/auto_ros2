use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct UInt8MultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: safe_drive::msg::U8Seq<0>,
}

extern "C" {
    fn std_msgs__msg__UInt8MultiArray__init(msg: *mut UInt8MultiArray) -> bool;
    fn std_msgs__msg__UInt8MultiArray__fini(msg: *mut UInt8MultiArray);
    fn std_msgs__msg__UInt8MultiArray__are_equal(lhs: *const UInt8MultiArray, rhs: *const UInt8MultiArray) -> bool;
    fn std_msgs__msg__UInt8MultiArray__Sequence__init(msg: *mut UInt8MultiArraySeqRaw, size: usize) -> bool;
    fn std_msgs__msg__UInt8MultiArray__Sequence__fini(msg: *mut UInt8MultiArraySeqRaw);
    fn std_msgs__msg__UInt8MultiArray__Sequence__are_equal(lhs: *const UInt8MultiArraySeqRaw, rhs: *const UInt8MultiArraySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt8MultiArray() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for UInt8MultiArray {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt8MultiArray()
        }
    }
}

impl PartialEq for UInt8MultiArray {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__UInt8MultiArray__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for UInt8MultiArraySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = UInt8MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = UInt8MultiArraySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__UInt8MultiArray__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl UInt8MultiArray {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__UInt8MultiArray__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for UInt8MultiArray {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__UInt8MultiArray__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct UInt8MultiArraySeqRaw {
    data: *mut UInt8MultiArray,
    size: size_t,
    capacity: size_t,
}

/// Sequence of UInt8MultiArray.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct UInt8MultiArraySeq<const N: usize> {
    data: *mut UInt8MultiArray,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> UInt8MultiArraySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: UInt8MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__UInt8MultiArray__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: UInt8MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[UInt8MultiArray] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [UInt8MultiArray] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, UInt8MultiArray> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, UInt8MultiArray> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for UInt8MultiArraySeq<N> {
    fn drop(&mut self) {
        let mut msg = UInt8MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__UInt8MultiArray__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for UInt8MultiArraySeq<N> {}
unsafe impl<const N: usize> Sync for UInt8MultiArraySeq<N> {}
