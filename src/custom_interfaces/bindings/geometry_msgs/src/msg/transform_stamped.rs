use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct TransformStamped {
    pub header: std_msgs::msg::Header,
    pub child_frame_id: safe_drive::msg::RosString<0>,
    pub transform: crate::msg::Transform,
}

extern "C" {
    fn geometry_msgs__msg__TransformStamped__init(msg: *mut TransformStamped) -> bool;
    fn geometry_msgs__msg__TransformStamped__fini(msg: *mut TransformStamped);
    fn geometry_msgs__msg__TransformStamped__are_equal(lhs: *const TransformStamped, rhs: *const TransformStamped) -> bool;
    fn geometry_msgs__msg__TransformStamped__Sequence__init(msg: *mut TransformStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__TransformStamped__Sequence__fini(msg: *mut TransformStampedSeqRaw);
    fn geometry_msgs__msg__TransformStamped__Sequence__are_equal(lhs: *const TransformStampedSeqRaw, rhs: *const TransformStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TransformStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TransformStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__TransformStamped()
        }
    }
}

impl PartialEq for TransformStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__TransformStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TransformStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TransformStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TransformStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__TransformStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TransformStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__TransformStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TransformStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__TransformStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TransformStampedSeqRaw {
    data: *mut TransformStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TransformStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TransformStampedSeq<const N: usize> {
    data: *mut TransformStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TransformStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TransformStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__TransformStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TransformStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TransformStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TransformStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TransformStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TransformStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TransformStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = TransformStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__TransformStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TransformStampedSeq<N> {}
unsafe impl<const N: usize> Sync for TransformStampedSeq<N> {}
