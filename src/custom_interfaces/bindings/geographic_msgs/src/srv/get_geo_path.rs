use safe_drive::{msg::{ServiceMsg, TypeSupport}, rcl::{self, size_t}};

extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetGeoPath() -> *const rcl::rosidl_service_type_support_t;
}

#[derive(Debug)]
pub struct GetGeoPath;

impl ServiceMsg for GetGeoPath {
    type Request = GetGeoPath_Request;
    type Response = GetGeoPath_Response;
    fn type_support() -> *const rcl::rosidl_service_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetGeoPath()
        }
    }
}


#[repr(C)]
#[derive(Debug)]
pub struct GetGeoPath_Request {
    pub start: crate::msg::GeoPoint,
    pub goal: crate::msg::GeoPoint,
}

extern "C" {
    fn geographic_msgs__srv__GetGeoPath_Request__init(msg: *mut GetGeoPath_Request) -> bool;
    fn geographic_msgs__srv__GetGeoPath_Request__fini(msg: *mut GetGeoPath_Request);
    fn geographic_msgs__srv__GetGeoPath_Request__are_equal(lhs: *const GetGeoPath_Request, rhs: *const GetGeoPath_Request) -> bool;
    fn geographic_msgs__srv__GetGeoPath_Request__Sequence__init(msg: *mut GetGeoPath_RequestSeqRaw, size: usize) -> bool;
    fn geographic_msgs__srv__GetGeoPath_Request__Sequence__fini(msg: *mut GetGeoPath_RequestSeqRaw);
    fn geographic_msgs__srv__GetGeoPath_Request__Sequence__are_equal(lhs: *const GetGeoPath_RequestSeqRaw, rhs: *const GetGeoPath_RequestSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeoPath_Request() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GetGeoPath_Request {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeoPath_Request()
        }
    }
}

impl PartialEq for GetGeoPath_Request {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__srv__GetGeoPath_Request__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GetGeoPath_RequestSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GetGeoPath_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GetGeoPath_RequestSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__srv__GetGeoPath_Request__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GetGeoPath_Request {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__srv__GetGeoPath_Request__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GetGeoPath_Request {
    fn drop(&mut self) {
        unsafe { geographic_msgs__srv__GetGeoPath_Request__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GetGeoPath_RequestSeqRaw {
    data: *mut GetGeoPath_Request,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GetGeoPath_Request.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GetGeoPath_RequestSeq<const N: usize> {
    data: *mut GetGeoPath_Request,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GetGeoPath_RequestSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GetGeoPath_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__srv__GetGeoPath_Request__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GetGeoPath_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GetGeoPath_Request] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GetGeoPath_Request] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GetGeoPath_Request> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GetGeoPath_Request> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GetGeoPath_RequestSeq<N> {
    fn drop(&mut self) {
        let mut msg = GetGeoPath_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__srv__GetGeoPath_Request__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GetGeoPath_RequestSeq<N> {}
unsafe impl<const N: usize> Sync for GetGeoPath_RequestSeq<N> {}

#[repr(C)]
#[derive(Debug)]
pub struct GetGeoPath_Response {
    pub success: bool,
    pub status: safe_drive::msg::RosString<0>,
    pub plan: crate::msg::GeoPath,
    pub network: unique_identifier_msgs::msg::UUID,
    pub start_seg: unique_identifier_msgs::msg::UUID,
    pub goal_seg: unique_identifier_msgs::msg::UUID,
    pub distance: f64,
}

extern "C" {
    fn geographic_msgs__srv__GetGeoPath_Response__init(msg: *mut GetGeoPath_Response) -> bool;
    fn geographic_msgs__srv__GetGeoPath_Response__fini(msg: *mut GetGeoPath_Response);
    fn geographic_msgs__srv__GetGeoPath_Response__are_equal(lhs: *const GetGeoPath_Response, rhs: *const GetGeoPath_Response) -> bool;
    fn geographic_msgs__srv__GetGeoPath_Response__Sequence__init(msg: *mut GetGeoPath_ResponseSeqRaw, size: usize) -> bool;
    fn geographic_msgs__srv__GetGeoPath_Response__Sequence__fini(msg: *mut GetGeoPath_ResponseSeqRaw);
    fn geographic_msgs__srv__GetGeoPath_Response__Sequence__are_equal(lhs: *const GetGeoPath_ResponseSeqRaw, rhs: *const GetGeoPath_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeoPath_Response() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GetGeoPath_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeoPath_Response()
        }
    }
}

impl PartialEq for GetGeoPath_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__srv__GetGeoPath_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GetGeoPath_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GetGeoPath_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GetGeoPath_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__srv__GetGeoPath_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GetGeoPath_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__srv__GetGeoPath_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GetGeoPath_Response {
    fn drop(&mut self) {
        unsafe { geographic_msgs__srv__GetGeoPath_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GetGeoPath_ResponseSeqRaw {
    data: *mut GetGeoPath_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GetGeoPath_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GetGeoPath_ResponseSeq<const N: usize> {
    data: *mut GetGeoPath_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GetGeoPath_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GetGeoPath_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__srv__GetGeoPath_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GetGeoPath_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GetGeoPath_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GetGeoPath_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GetGeoPath_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GetGeoPath_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GetGeoPath_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = GetGeoPath_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__srv__GetGeoPath_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GetGeoPath_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for GetGeoPath_ResponseSeq<N> {}
