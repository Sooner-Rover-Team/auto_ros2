use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

extern "C" {
    fn geographic_msgs__msg__UpdateGeographicMap_Response__init(msg: *mut UpdateGeographicMap_Response) -> bool;
    fn geographic_msgs__msg__UpdateGeographicMap_Response__fini(msg: *mut UpdateGeographicMap_Response);
    fn geographic_msgs__msg__UpdateGeographicMap_Response__are_equal(lhs: *const UpdateGeographicMap_Response, rhs: *const UpdateGeographicMap_Response) -> bool;
    fn geographic_msgs__msg__UpdateGeographicMap_Response__Sequence__init(msg: *mut UpdateGeographicMap_ResponseSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__UpdateGeographicMap_Response__Sequence__fini(msg: *mut UpdateGeographicMap_ResponseSeqRaw);
    fn geographic_msgs__msg__UpdateGeographicMap_Response__Sequence__are_equal(lhs: *const UpdateGeographicMap_ResponseSeqRaw, rhs: *const UpdateGeographicMap_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__UpdateGeographicMap_Response() -> *const rcl::rosidl_message_type_support_t;
}

#[repr(C)]
#[derive(Debug)]
pub struct UpdateGeographicMap_Response {
    pub success: bool,
    pub status: safe_drive::msg::RosString<0>,
}

impl UpdateGeographicMap_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__UpdateGeographicMap_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for UpdateGeographicMap_Response {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__UpdateGeographicMap_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct UpdateGeographicMap_ResponseSeqRaw {
    data: *mut UpdateGeographicMap_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of UpdateGeographicMap_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct UpdateGeographicMap_ResponseSeq<const N: usize> {
    data: *mut UpdateGeographicMap_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> UpdateGeographicMap_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: UpdateGeographicMap_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__UpdateGeographicMap_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: UpdateGeographicMap_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[UpdateGeographicMap_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [UpdateGeographicMap_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, UpdateGeographicMap_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, UpdateGeographicMap_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for UpdateGeographicMap_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = UpdateGeographicMap_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__UpdateGeographicMap_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for UpdateGeographicMap_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for UpdateGeographicMap_ResponseSeq<N> {}

impl TypeSupport for UpdateGeographicMap_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__UpdateGeographicMap_Response()
        }
    }
}

impl PartialEq for UpdateGeographicMap_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__UpdateGeographicMap_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for UpdateGeographicMap_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = UpdateGeographicMap_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = UpdateGeographicMap_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__UpdateGeographicMap_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

