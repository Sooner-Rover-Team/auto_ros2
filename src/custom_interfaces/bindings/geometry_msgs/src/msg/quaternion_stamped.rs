use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct QuaternionStamped {
    pub header: std_msgs::msg::Header,
    pub quaternion: crate::msg::Quaternion,
}

extern "C" {
    fn geometry_msgs__msg__QuaternionStamped__init(msg: *mut QuaternionStamped) -> bool;
    fn geometry_msgs__msg__QuaternionStamped__fini(msg: *mut QuaternionStamped);
    fn geometry_msgs__msg__QuaternionStamped__are_equal(lhs: *const QuaternionStamped, rhs: *const QuaternionStamped) -> bool;
    fn geometry_msgs__msg__QuaternionStamped__Sequence__init(msg: *mut QuaternionStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__QuaternionStamped__Sequence__fini(msg: *mut QuaternionStampedSeqRaw);
    fn geometry_msgs__msg__QuaternionStamped__Sequence__are_equal(lhs: *const QuaternionStampedSeqRaw, rhs: *const QuaternionStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__QuaternionStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for QuaternionStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__QuaternionStamped()
        }
    }
}

impl PartialEq for QuaternionStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__QuaternionStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for QuaternionStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = QuaternionStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = QuaternionStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__QuaternionStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl QuaternionStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__QuaternionStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for QuaternionStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__QuaternionStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct QuaternionStampedSeqRaw {
    data: *mut QuaternionStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of QuaternionStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct QuaternionStampedSeq<const N: usize> {
    data: *mut QuaternionStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> QuaternionStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: QuaternionStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__QuaternionStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: QuaternionStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[QuaternionStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [QuaternionStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, QuaternionStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, QuaternionStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for QuaternionStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = QuaternionStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__QuaternionStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for QuaternionStampedSeq<N> {}
unsafe impl<const N: usize> Sync for QuaternionStampedSeq<N> {}
