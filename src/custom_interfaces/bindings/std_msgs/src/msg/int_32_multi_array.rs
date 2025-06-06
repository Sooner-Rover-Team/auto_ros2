use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Int32MultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: safe_drive::msg::I32Seq<0>,
}

extern "C" {
    fn std_msgs__msg__Int32MultiArray__init(msg: *mut Int32MultiArray) -> bool;
    fn std_msgs__msg__Int32MultiArray__fini(msg: *mut Int32MultiArray);
    fn std_msgs__msg__Int32MultiArray__are_equal(lhs: *const Int32MultiArray, rhs: *const Int32MultiArray) -> bool;
    fn std_msgs__msg__Int32MultiArray__Sequence__init(msg: *mut Int32MultiArraySeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Int32MultiArray__Sequence__fini(msg: *mut Int32MultiArraySeqRaw);
    fn std_msgs__msg__Int32MultiArray__Sequence__are_equal(lhs: *const Int32MultiArraySeqRaw, rhs: *const Int32MultiArraySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32MultiArray() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Int32MultiArray {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32MultiArray()
        }
    }
}

impl PartialEq for Int32MultiArray {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Int32MultiArray__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Int32MultiArraySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Int32MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Int32MultiArraySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Int32MultiArray__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Int32MultiArray {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Int32MultiArray__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Int32MultiArray {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Int32MultiArray__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Int32MultiArraySeqRaw {
    data: *mut Int32MultiArray,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Int32MultiArray.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Int32MultiArraySeq<const N: usize> {
    data: *mut Int32MultiArray,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Int32MultiArraySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Int32MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Int32MultiArray__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Int32MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Int32MultiArray] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Int32MultiArray] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Int32MultiArray> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Int32MultiArray> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Int32MultiArraySeq<N> {
    fn drop(&mut self) {
        let mut msg = Int32MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Int32MultiArray__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Int32MultiArraySeq<N> {}
unsafe impl<const N: usize> Sync for Int32MultiArraySeq<N> {}
