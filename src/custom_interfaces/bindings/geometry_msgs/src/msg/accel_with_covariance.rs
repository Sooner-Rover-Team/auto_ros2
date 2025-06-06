use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

type double__36 = [f64; 36];

#[repr(C)]
#[derive(Debug)]
pub struct AccelWithCovariance {
    pub accel: crate::msg::Accel,
    pub covariance: double__36,
}

extern "C" {
    fn geometry_msgs__msg__AccelWithCovariance__init(msg: *mut AccelWithCovariance) -> bool;
    fn geometry_msgs__msg__AccelWithCovariance__fini(msg: *mut AccelWithCovariance);
    fn geometry_msgs__msg__AccelWithCovariance__are_equal(lhs: *const AccelWithCovariance, rhs: *const AccelWithCovariance) -> bool;
    fn geometry_msgs__msg__AccelWithCovariance__Sequence__init(msg: *mut AccelWithCovarianceSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__AccelWithCovariance__Sequence__fini(msg: *mut AccelWithCovarianceSeqRaw);
    fn geometry_msgs__msg__AccelWithCovariance__Sequence__are_equal(lhs: *const AccelWithCovarianceSeqRaw, rhs: *const AccelWithCovarianceSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__AccelWithCovariance() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for AccelWithCovariance {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__AccelWithCovariance()
        }
    }
}

impl PartialEq for AccelWithCovariance {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__AccelWithCovariance__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for AccelWithCovarianceSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = AccelWithCovarianceSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = AccelWithCovarianceSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__AccelWithCovariance__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl AccelWithCovariance {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__AccelWithCovariance__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for AccelWithCovariance {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__AccelWithCovariance__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct AccelWithCovarianceSeqRaw {
    data: *mut AccelWithCovariance,
    size: size_t,
    capacity: size_t,
}

/// Sequence of AccelWithCovariance.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct AccelWithCovarianceSeq<const N: usize> {
    data: *mut AccelWithCovariance,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> AccelWithCovarianceSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: AccelWithCovarianceSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__AccelWithCovariance__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: AccelWithCovarianceSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[AccelWithCovariance] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [AccelWithCovariance] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, AccelWithCovariance> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, AccelWithCovariance> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for AccelWithCovarianceSeq<N> {
    fn drop(&mut self) {
        let mut msg = AccelWithCovarianceSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__AccelWithCovariance__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for AccelWithCovarianceSeq<N> {}
unsafe impl<const N: usize> Sync for AccelWithCovarianceSeq<N> {}
