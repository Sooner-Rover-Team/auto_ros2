use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

extern "C" {
    fn custom_interfaces__msg__GnssToMap_Response__init(msg: *mut GnssToMap_Response) -> bool;
    fn custom_interfaces__msg__GnssToMap_Response__fini(msg: *mut GnssToMap_Response);
    fn custom_interfaces__msg__GnssToMap_Response__are_equal(lhs: *const GnssToMap_Response, rhs: *const GnssToMap_Response) -> bool;
    fn custom_interfaces__msg__GnssToMap_Response__Sequence__init(msg: *mut GnssToMap_ResponseSeqRaw, size: usize) -> bool;
    fn custom_interfaces__msg__GnssToMap_Response__Sequence__fini(msg: *mut GnssToMap_ResponseSeqRaw);
    fn custom_interfaces__msg__GnssToMap_Response__Sequence__are_equal(lhs: *const GnssToMap_ResponseSeqRaw, rhs: *const GnssToMap_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__GnssToMap_Response() -> *const rcl::rosidl_message_type_support_t;
}

#[repr(C)]
#[derive(Debug)]
pub struct GnssToMap_Response {
    pub success: bool,
    pub point: geometry_msgs::msg::point::Point,
}

impl GnssToMap_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__GnssToMap_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GnssToMap_Response {
    fn drop(&mut self) {
        unsafe { custom_interfaces__msg__GnssToMap_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GnssToMap_ResponseSeqRaw {
    data: *mut GnssToMap_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GnssToMap_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GnssToMap_ResponseSeq<const N: usize> {
    data: *mut GnssToMap_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GnssToMap_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GnssToMap_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__GnssToMap_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GnssToMap_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GnssToMap_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GnssToMap_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GnssToMap_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GnssToMap_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GnssToMap_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = GnssToMap_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__msg__GnssToMap_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GnssToMap_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for GnssToMap_ResponseSeq<N> {}

impl TypeSupport for GnssToMap_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__GnssToMap_Response()
        }
    }
}

impl PartialEq for GnssToMap_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__msg__GnssToMap_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GnssToMap_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GnssToMap_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GnssToMap_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__msg__GnssToMap_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

