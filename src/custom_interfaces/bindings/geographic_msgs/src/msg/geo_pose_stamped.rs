use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct GeoPoseStamped {
    pub header: std_msgs::msg::Header,
    pub pose: crate::msg::GeoPose,
}

extern "C" {
    fn geographic_msgs__msg__GeoPoseStamped__init(msg: *mut GeoPoseStamped) -> bool;
    fn geographic_msgs__msg__GeoPoseStamped__fini(msg: *mut GeoPoseStamped);
    fn geographic_msgs__msg__GeoPoseStamped__are_equal(lhs: *const GeoPoseStamped, rhs: *const GeoPoseStamped) -> bool;
    fn geographic_msgs__msg__GeoPoseStamped__Sequence__init(msg: *mut GeoPoseStampedSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__GeoPoseStamped__Sequence__fini(msg: *mut GeoPoseStampedSeqRaw);
    fn geographic_msgs__msg__GeoPoseStamped__Sequence__are_equal(lhs: *const GeoPoseStampedSeqRaw, rhs: *const GeoPoseStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoseStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GeoPoseStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoseStamped()
        }
    }
}

impl PartialEq for GeoPoseStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__GeoPoseStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GeoPoseStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GeoPoseStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GeoPoseStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__GeoPoseStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GeoPoseStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeoPoseStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GeoPoseStamped {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__GeoPoseStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GeoPoseStampedSeqRaw {
    data: *mut GeoPoseStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GeoPoseStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GeoPoseStampedSeq<const N: usize> {
    data: *mut GeoPoseStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GeoPoseStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GeoPoseStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeoPoseStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GeoPoseStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GeoPoseStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GeoPoseStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GeoPoseStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GeoPoseStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GeoPoseStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = GeoPoseStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__GeoPoseStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GeoPoseStampedSeq<N> {}
unsafe impl<const N: usize> Sync for GeoPoseStampedSeq<N> {}
