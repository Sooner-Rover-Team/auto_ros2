use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

extern "C" {
    fn custom_interfaces__msg__GnssToMap_Request__init(msg: *mut GnssToMap_Request) -> bool;
    fn custom_interfaces__msg__GnssToMap_Request__fini(msg: *mut GnssToMap_Request);
    fn custom_interfaces__msg__GnssToMap_Request__are_equal(lhs: *const GnssToMap_Request, rhs: *const GnssToMap_Request) -> bool;
    fn custom_interfaces__msg__GnssToMap_Request__Sequence__init(msg: *mut GnssToMap_RequestSeqRaw, size: usize) -> bool;
    fn custom_interfaces__msg__GnssToMap_Request__Sequence__fini(msg: *mut GnssToMap_RequestSeqRaw);
    fn custom_interfaces__msg__GnssToMap_Request__Sequence__are_equal(lhs: *const GnssToMap_RequestSeqRaw, rhs: *const GnssToMap_RequestSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__GnssToMap_Request() -> *const rcl::rosidl_message_type_support_t;
}

#[repr(C)]
#[derive(Debug)]
pub struct GnssToMap_Request {
    pub gnss_coord_to_convert: geographic_msgs::msg::geo_point::GeoPoint,
}

impl GnssToMap_Request {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__GnssToMap_Request__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GnssToMap_Request {
    fn drop(&mut self) {
        unsafe { custom_interfaces__msg__GnssToMap_Request__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GnssToMap_RequestSeqRaw {
    data: *mut GnssToMap_Request,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GnssToMap_Request.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GnssToMap_RequestSeq<const N: usize> {
    data: *mut GnssToMap_Request,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GnssToMap_RequestSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GnssToMap_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__GnssToMap_Request__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GnssToMap_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GnssToMap_Request] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GnssToMap_Request] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GnssToMap_Request> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GnssToMap_Request> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GnssToMap_RequestSeq<N> {
    fn drop(&mut self) {
        let mut msg = GnssToMap_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__msg__GnssToMap_Request__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GnssToMap_RequestSeq<N> {}
unsafe impl<const N: usize> Sync for GnssToMap_RequestSeq<N> {}

impl TypeSupport for GnssToMap_Request {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__GnssToMap_Request()
        }
    }
}

impl PartialEq for GnssToMap_Request {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__msg__GnssToMap_Request__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GnssToMap_RequestSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GnssToMap_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GnssToMap_RequestSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__msg__GnssToMap_Request__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

