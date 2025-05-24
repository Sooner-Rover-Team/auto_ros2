use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct GeoPoseWithCovarianceStamped {
    pub header: std_msgs::msg::Header,
    pub pose: crate::msg::GeoPoseWithCovariance,
}

extern "C" {
    fn geographic_msgs__msg__GeoPoseWithCovarianceStamped__init(msg: *mut GeoPoseWithCovarianceStamped) -> bool;
    fn geographic_msgs__msg__GeoPoseWithCovarianceStamped__fini(msg: *mut GeoPoseWithCovarianceStamped);
    fn geographic_msgs__msg__GeoPoseWithCovarianceStamped__are_equal(lhs: *const GeoPoseWithCovarianceStamped, rhs: *const GeoPoseWithCovarianceStamped) -> bool;
    fn geographic_msgs__msg__GeoPoseWithCovarianceStamped__Sequence__init(msg: *mut GeoPoseWithCovarianceStampedSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__GeoPoseWithCovarianceStamped__Sequence__fini(msg: *mut GeoPoseWithCovarianceStampedSeqRaw);
    fn geographic_msgs__msg__GeoPoseWithCovarianceStamped__Sequence__are_equal(lhs: *const GeoPoseWithCovarianceStampedSeqRaw, rhs: *const GeoPoseWithCovarianceStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoseWithCovarianceStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GeoPoseWithCovarianceStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoseWithCovarianceStamped()
        }
    }
}

impl PartialEq for GeoPoseWithCovarianceStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__GeoPoseWithCovarianceStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GeoPoseWithCovarianceStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GeoPoseWithCovarianceStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GeoPoseWithCovarianceStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__GeoPoseWithCovarianceStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GeoPoseWithCovarianceStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeoPoseWithCovarianceStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GeoPoseWithCovarianceStamped {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__GeoPoseWithCovarianceStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GeoPoseWithCovarianceStampedSeqRaw {
    data: *mut GeoPoseWithCovarianceStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GeoPoseWithCovarianceStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GeoPoseWithCovarianceStampedSeq<const N: usize> {
    data: *mut GeoPoseWithCovarianceStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GeoPoseWithCovarianceStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GeoPoseWithCovarianceStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeoPoseWithCovarianceStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GeoPoseWithCovarianceStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GeoPoseWithCovarianceStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GeoPoseWithCovarianceStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GeoPoseWithCovarianceStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GeoPoseWithCovarianceStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GeoPoseWithCovarianceStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = GeoPoseWithCovarianceStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__GeoPoseWithCovarianceStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GeoPoseWithCovarianceStampedSeq<N> {}
unsafe impl<const N: usize> Sync for GeoPoseWithCovarianceStampedSeq<N> {}
