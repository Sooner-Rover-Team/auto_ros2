use safe_drive::{msg::{ServiceMsg, TypeSupport}, rcl::{self, size_t}};

extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__srv__Lights() -> *const rcl::rosidl_service_type_support_t;
}

#[derive(Debug)]
pub struct Lights;

impl ServiceMsg for Lights {
    type Request = Lights_Request;
    type Response = Lights_Response;
    fn type_support() -> *const rcl::rosidl_service_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__srv__Lights()
        }
    }
}


#[repr(C)]
#[derive(Debug)]
pub struct Lights_Request {
    pub red: u8,
    pub blue: u8,
    pub green: u8,
    pub flashing: bool,
}

extern "C" {
    fn custom_interfaces__srv__Lights_Request__init(msg: *mut Lights_Request) -> bool;
    fn custom_interfaces__srv__Lights_Request__fini(msg: *mut Lights_Request);
    fn custom_interfaces__srv__Lights_Request__are_equal(lhs: *const Lights_Request, rhs: *const Lights_Request) -> bool;
    fn custom_interfaces__srv__Lights_Request__Sequence__init(msg: *mut Lights_RequestSeqRaw, size: usize) -> bool;
    fn custom_interfaces__srv__Lights_Request__Sequence__fini(msg: *mut Lights_RequestSeqRaw);
    fn custom_interfaces__srv__Lights_Request__Sequence__are_equal(lhs: *const Lights_RequestSeqRaw, rhs: *const Lights_RequestSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__srv__Lights_Request() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Lights_Request {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__srv__Lights_Request()
        }
    }
}

impl PartialEq for Lights_Request {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__srv__Lights_Request__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Lights_RequestSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Lights_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Lights_RequestSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__srv__Lights_Request__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Lights_Request {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__srv__Lights_Request__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Lights_Request {
    fn drop(&mut self) {
        unsafe { custom_interfaces__srv__Lights_Request__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Lights_RequestSeqRaw {
    data: *mut Lights_Request,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Lights_Request.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Lights_RequestSeq<const N: usize> {
    data: *mut Lights_Request,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Lights_RequestSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Lights_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__srv__Lights_Request__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Lights_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Lights_Request] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Lights_Request] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Lights_Request> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Lights_Request> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Lights_RequestSeq<N> {
    fn drop(&mut self) {
        let mut msg = Lights_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__srv__Lights_Request__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Lights_RequestSeq<N> {}
unsafe impl<const N: usize> Sync for Lights_RequestSeq<N> {}

#[repr(C)]
#[derive(Debug)]
pub struct Lights_Response {
    pub success: bool,
}

extern "C" {
    fn custom_interfaces__srv__Lights_Response__init(msg: *mut Lights_Response) -> bool;
    fn custom_interfaces__srv__Lights_Response__fini(msg: *mut Lights_Response);
    fn custom_interfaces__srv__Lights_Response__are_equal(lhs: *const Lights_Response, rhs: *const Lights_Response) -> bool;
    fn custom_interfaces__srv__Lights_Response__Sequence__init(msg: *mut Lights_ResponseSeqRaw, size: usize) -> bool;
    fn custom_interfaces__srv__Lights_Response__Sequence__fini(msg: *mut Lights_ResponseSeqRaw);
    fn custom_interfaces__srv__Lights_Response__Sequence__are_equal(lhs: *const Lights_ResponseSeqRaw, rhs: *const Lights_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__srv__Lights_Response() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Lights_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__srv__Lights_Response()
        }
    }
}

impl PartialEq for Lights_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__srv__Lights_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Lights_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Lights_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Lights_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__srv__Lights_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Lights_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__srv__Lights_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Lights_Response {
    fn drop(&mut self) {
        unsafe { custom_interfaces__srv__Lights_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Lights_ResponseSeqRaw {
    data: *mut Lights_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Lights_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Lights_ResponseSeq<const N: usize> {
    data: *mut Lights_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Lights_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Lights_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__srv__Lights_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Lights_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Lights_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Lights_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Lights_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Lights_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Lights_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = Lights_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__srv__Lights_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Lights_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for Lights_ResponseSeq<N> {}
