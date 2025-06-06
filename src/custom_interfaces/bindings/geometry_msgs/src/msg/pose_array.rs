use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct PoseArray {
    pub header: std_msgs::msg::Header,
    pub poses: crate::msg::PoseSeq<0>,
}

extern "C" {
    fn geometry_msgs__msg__PoseArray__init(msg: *mut PoseArray) -> bool;
    fn geometry_msgs__msg__PoseArray__fini(msg: *mut PoseArray);
    fn geometry_msgs__msg__PoseArray__are_equal(lhs: *const PoseArray, rhs: *const PoseArray) -> bool;
    fn geometry_msgs__msg__PoseArray__Sequence__init(msg: *mut PoseArraySeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__PoseArray__Sequence__fini(msg: *mut PoseArraySeqRaw);
    fn geometry_msgs__msg__PoseArray__Sequence__are_equal(lhs: *const PoseArraySeqRaw, rhs: *const PoseArraySeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PoseArray() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for PoseArray {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PoseArray()
        }
    }
}

impl PartialEq for PoseArray {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__PoseArray__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for PoseArraySeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = PoseArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = PoseArraySeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__PoseArray__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl PoseArray {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__PoseArray__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for PoseArray {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__PoseArray__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct PoseArraySeqRaw {
    data: *mut PoseArray,
    size: size_t,
    capacity: size_t,
}

/// Sequence of PoseArray.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct PoseArraySeq<const N: usize> {
    data: *mut PoseArray,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> PoseArraySeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: PoseArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__PoseArray__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: PoseArraySeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[PoseArray] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [PoseArray] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, PoseArray> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, PoseArray> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for PoseArraySeq<N> {
    fn drop(&mut self) {
        let mut msg = PoseArraySeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__PoseArray__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for PoseArraySeq<N> {}
unsafe impl<const N: usize> Sync for PoseArraySeq<N> {}
