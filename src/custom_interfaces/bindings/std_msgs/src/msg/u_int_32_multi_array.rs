use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct UInt32MultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: safe_drive::msg::U32Seq<0>,
}

extern "C" {
    fn std_msgs__msg__UInt32MultiArray__init(msg: *mut UInt32MultiArray) -> bool;
    fn std_msgs__msg__UInt32MultiArray__fini(msg: *mut UInt32MultiArray);
    fn std_msgs__msg__UInt32MultiArray__are_equal(lhs: *const UInt32MultiArray, rhs: *const UInt32MultiArray) -> bool;
    fn std_msgs__msg__UInt32MultiArray__Sequence__init(msg: *mut UInt32MultiArraySeqRaw, size: usize) -> bool;
    fn std_msgs__msg__UInt32MultiArray__Sequence__fini(msg: *mut UInt32MultiArraySeqRaw);
    fn std_msgs__msg__UInt32MultiArray__Sequence__are_equal(lhs: *const UInt32MultiArraySeqRaw, rhs: *const UInt32MultiArraySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt32MultiArray() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for UInt32MultiArray {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__UInt32MultiArray()
        }
    }
}

impl PartialEq for UInt32MultiArray {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__UInt32MultiArray__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for UInt32MultiArraySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = UInt32MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = UInt32MultiArraySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__UInt32MultiArray__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl UInt32MultiArray {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__UInt32MultiArray__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for UInt32MultiArray {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__UInt32MultiArray__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct UInt32MultiArraySeqRaw {
    data: *mut UInt32MultiArray,
    size: size_t,
    capacity: size_t,
}

/// Sequence of UInt32MultiArray.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct UInt32MultiArraySeq<const N: usize> {
    data: *mut UInt32MultiArray,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> UInt32MultiArraySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: UInt32MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__UInt32MultiArray__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: UInt32MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[UInt32MultiArray] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [UInt32MultiArray] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, UInt32MultiArray> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, UInt32MultiArray> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for UInt32MultiArraySeq<N> {
    fn drop(&mut self) {
        let mut msg = UInt32MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__UInt32MultiArray__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for UInt32MultiArraySeq<N> {}
unsafe impl<const N: usize> Sync for UInt32MultiArraySeq<N> {}
