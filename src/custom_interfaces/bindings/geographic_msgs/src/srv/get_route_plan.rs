use safe_drive::{msg::{ServiceMsg, TypeSupport}, rcl::{self, size_t}};

extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetRoutePlan() -> *const rcl::rosidl_service_type_support_t;
}

#[derive(Debug)]
pub struct GetRoutePlan;

impl ServiceMsg for GetRoutePlan {
    type Request = GetRoutePlan_Request;
    type Response = GetRoutePlan_Response;
    fn type_support() -> *const rcl::rosidl_service_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetRoutePlan()
        }
    }
}


#[repr(C)]
#[derive(Debug)]
pub struct GetRoutePlan_Request {
    pub network: unique_identifier_msgs::msg::UUID,
    pub start: unique_identifier_msgs::msg::UUID,
    pub goal: unique_identifier_msgs::msg::UUID,
}

extern "C" {
    fn geographic_msgs__srv__GetRoutePlan_Request__init(msg: *mut GetRoutePlan_Request) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Request__fini(msg: *mut GetRoutePlan_Request);
    fn geographic_msgs__srv__GetRoutePlan_Request__are_equal(lhs: *const GetRoutePlan_Request, rhs: *const GetRoutePlan_Request) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Request__Sequence__init(msg: *mut GetRoutePlan_RequestSeqRaw, size: usize) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Request__Sequence__fini(msg: *mut GetRoutePlan_RequestSeqRaw);
    fn geographic_msgs__srv__GetRoutePlan_Request__Sequence__are_equal(lhs: *const GetRoutePlan_RequestSeqRaw, rhs: *const GetRoutePlan_RequestSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Request() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GetRoutePlan_Request {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Request()
        }
    }
}

impl PartialEq for GetRoutePlan_Request {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__srv__GetRoutePlan_Request__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GetRoutePlan_RequestSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GetRoutePlan_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GetRoutePlan_RequestSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__srv__GetRoutePlan_Request__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GetRoutePlan_Request {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__srv__GetRoutePlan_Request__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GetRoutePlan_Request {
    fn drop(&mut self) {
        unsafe { geographic_msgs__srv__GetRoutePlan_Request__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GetRoutePlan_RequestSeqRaw {
    data: *mut GetRoutePlan_Request,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GetRoutePlan_Request.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GetRoutePlan_RequestSeq<const N: usize> {
    data: *mut GetRoutePlan_Request,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GetRoutePlan_RequestSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GetRoutePlan_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__srv__GetRoutePlan_Request__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GetRoutePlan_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GetRoutePlan_Request] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GetRoutePlan_Request] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GetRoutePlan_Request> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GetRoutePlan_Request> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GetRoutePlan_RequestSeq<N> {
    fn drop(&mut self) {
        let mut msg = GetRoutePlan_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__srv__GetRoutePlan_Request__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GetRoutePlan_RequestSeq<N> {}
unsafe impl<const N: usize> Sync for GetRoutePlan_RequestSeq<N> {}

#[repr(C)]
#[derive(Debug)]
pub struct GetRoutePlan_Response {
    pub success: bool,
    pub status: safe_drive::msg::RosString<0>,
    pub plan: crate::msg::RoutePath,
}

extern "C" {
    fn geographic_msgs__srv__GetRoutePlan_Response__init(msg: *mut GetRoutePlan_Response) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Response__fini(msg: *mut GetRoutePlan_Response);
    fn geographic_msgs__srv__GetRoutePlan_Response__are_equal(lhs: *const GetRoutePlan_Response, rhs: *const GetRoutePlan_Response) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Response__Sequence__init(msg: *mut GetRoutePlan_ResponseSeqRaw, size: usize) -> bool;
    fn geographic_msgs__srv__GetRoutePlan_Response__Sequence__fini(msg: *mut GetRoutePlan_ResponseSeqRaw);
    fn geographic_msgs__srv__GetRoutePlan_Response__Sequence__are_equal(lhs: *const GetRoutePlan_ResponseSeqRaw, rhs: *const GetRoutePlan_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Response() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GetRoutePlan_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Response()
        }
    }
}

impl PartialEq for GetRoutePlan_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__srv__GetRoutePlan_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GetRoutePlan_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GetRoutePlan_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GetRoutePlan_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__srv__GetRoutePlan_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GetRoutePlan_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__srv__GetRoutePlan_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GetRoutePlan_Response {
    fn drop(&mut self) {
        unsafe { geographic_msgs__srv__GetRoutePlan_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GetRoutePlan_ResponseSeqRaw {
    data: *mut GetRoutePlan_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GetRoutePlan_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GetRoutePlan_ResponseSeq<const N: usize> {
    data: *mut GetRoutePlan_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GetRoutePlan_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GetRoutePlan_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__srv__GetRoutePlan_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GetRoutePlan_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GetRoutePlan_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GetRoutePlan_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GetRoutePlan_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GetRoutePlan_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GetRoutePlan_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = GetRoutePlan_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__srv__GetRoutePlan_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GetRoutePlan_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for GetRoutePlan_ResponseSeq<N> {}
