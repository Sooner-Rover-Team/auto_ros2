use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct GeoPoint {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

extern "C" {
    fn geographic_msgs__msg__GeoPoint__init(msg: *mut GeoPoint) -> bool;
    fn geographic_msgs__msg__GeoPoint__fini(msg: *mut GeoPoint);
    fn geographic_msgs__msg__GeoPoint__are_equal(lhs: *const GeoPoint, rhs: *const GeoPoint) -> bool;
    fn geographic_msgs__msg__GeoPoint__Sequence__init(msg: *mut GeoPointSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__GeoPoint__Sequence__fini(msg: *mut GeoPointSeqRaw);
    fn geographic_msgs__msg__GeoPoint__Sequence__are_equal(lhs: *const GeoPointSeqRaw, rhs: *const GeoPointSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoint() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GeoPoint {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoint()
        }
    }
}

impl PartialEq for GeoPoint {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__GeoPoint__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GeoPointSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GeoPointSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GeoPointSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__GeoPoint__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GeoPoint {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeoPoint__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GeoPoint {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__GeoPoint__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GeoPointSeqRaw {
    data: *mut GeoPoint,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GeoPoint.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GeoPointSeq<const N: usize> {
    data: *mut GeoPoint,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GeoPointSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GeoPointSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeoPoint__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GeoPointSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GeoPoint] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GeoPoint] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GeoPoint> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GeoPoint> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GeoPointSeq<N> {
    fn drop(&mut self) {
        let mut msg = GeoPointSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__GeoPoint__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GeoPointSeq<N> {}
unsafe impl<const N: usize> Sync for GeoPointSeq<N> {}
