use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Float32MultiArray {
    pub layout: crate::msg::MultiArrayLayout,
    pub data: safe_drive::msg::F32Seq<0>,
}

extern "C" {
    fn std_msgs__msg__Float32MultiArray__init(msg: *mut Float32MultiArray) -> bool;
    fn std_msgs__msg__Float32MultiArray__fini(msg: *mut Float32MultiArray);
    fn std_msgs__msg__Float32MultiArray__are_equal(lhs: *const Float32MultiArray, rhs: *const Float32MultiArray) -> bool;
    fn std_msgs__msg__Float32MultiArray__Sequence__init(msg: *mut Float32MultiArraySeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Float32MultiArray__Sequence__fini(msg: *mut Float32MultiArraySeqRaw);
    fn std_msgs__msg__Float32MultiArray__Sequence__are_equal(lhs: *const Float32MultiArraySeqRaw, rhs: *const Float32MultiArraySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Float32MultiArray() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Float32MultiArray {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Float32MultiArray()
        }
    }
}

impl PartialEq for Float32MultiArray {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Float32MultiArray__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Float32MultiArraySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Float32MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Float32MultiArraySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Float32MultiArray__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Float32MultiArray {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Float32MultiArray__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Float32MultiArray {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Float32MultiArray__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Float32MultiArraySeqRaw {
    data: *mut Float32MultiArray,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Float32MultiArray.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Float32MultiArraySeq<const N: usize> {
    data: *mut Float32MultiArray,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Float32MultiArraySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Float32MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Float32MultiArray__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Float32MultiArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Float32MultiArray] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Float32MultiArray] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Float32MultiArray> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Float32MultiArray> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Float32MultiArraySeq<N> {
    fn drop(&mut self) {
        let mut msg = Float32MultiArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Float32MultiArray__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Float32MultiArraySeq<N> {}
unsafe impl<const N: usize> Sync for Float32MultiArraySeq<N> {}
