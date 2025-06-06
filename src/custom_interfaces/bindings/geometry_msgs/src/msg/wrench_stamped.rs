use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct WrenchStamped {
    pub header: std_msgs::msg::Header,
    pub wrench: crate::msg::Wrench,
}

extern "C" {
    fn geometry_msgs__msg__WrenchStamped__init(msg: *mut WrenchStamped) -> bool;
    fn geometry_msgs__msg__WrenchStamped__fini(msg: *mut WrenchStamped);
    fn geometry_msgs__msg__WrenchStamped__are_equal(lhs: *const WrenchStamped, rhs: *const WrenchStamped) -> bool;
    fn geometry_msgs__msg__WrenchStamped__Sequence__init(msg: *mut WrenchStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__WrenchStamped__Sequence__fini(msg: *mut WrenchStampedSeqRaw);
    fn geometry_msgs__msg__WrenchStamped__Sequence__are_equal(lhs: *const WrenchStampedSeqRaw, rhs: *const WrenchStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__WrenchStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for WrenchStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__WrenchStamped()
        }
    }
}

impl PartialEq for WrenchStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__WrenchStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for WrenchStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = WrenchStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = WrenchStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__WrenchStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl WrenchStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__WrenchStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for WrenchStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__WrenchStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct WrenchStampedSeqRaw {
    data: *mut WrenchStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of WrenchStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct WrenchStampedSeq<const N: usize> {
    data: *mut WrenchStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> WrenchStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: WrenchStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__WrenchStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: WrenchStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[WrenchStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [WrenchStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, WrenchStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, WrenchStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for WrenchStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = WrenchStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__WrenchStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for WrenchStampedSeq<N> {}
unsafe impl<const N: usize> Sync for WrenchStampedSeq<N> {}
