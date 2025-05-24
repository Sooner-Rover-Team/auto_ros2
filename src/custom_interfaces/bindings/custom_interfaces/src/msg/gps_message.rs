use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct GpsMessage {
    pub lat: f64,
    pub lon: f64,
    pub height: f64,
    pub error_mm: f64,
    pub time_of_week: u32,
}

extern "C" {
    fn custom_interfaces__msg__GpsMessage__init(msg: *mut GpsMessage) -> bool;
    fn custom_interfaces__msg__GpsMessage__fini(msg: *mut GpsMessage);
    fn custom_interfaces__msg__GpsMessage__are_equal(lhs: *const GpsMessage, rhs: *const GpsMessage) -> bool;
    fn custom_interfaces__msg__GpsMessage__Sequence__init(msg: *mut GpsMessageSeqRaw, size: usize) -> bool;
    fn custom_interfaces__msg__GpsMessage__Sequence__fini(msg: *mut GpsMessageSeqRaw);
    fn custom_interfaces__msg__GpsMessage__Sequence__are_equal(lhs: *const GpsMessageSeqRaw, rhs: *const GpsMessageSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__GpsMessage() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GpsMessage {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__GpsMessage()
        }
    }
}

impl PartialEq for GpsMessage {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__msg__GpsMessage__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GpsMessageSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GpsMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GpsMessageSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__msg__GpsMessage__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GpsMessage {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__GpsMessage__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GpsMessage {
    fn drop(&mut self) {
        unsafe { custom_interfaces__msg__GpsMessage__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GpsMessageSeqRaw {
    data: *mut GpsMessage,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GpsMessage.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GpsMessageSeq<const N: usize> {
    data: *mut GpsMessage,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GpsMessageSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GpsMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__GpsMessage__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GpsMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GpsMessage] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GpsMessage] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GpsMessage> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GpsMessage> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GpsMessageSeq<N> {
    fn drop(&mut self) {
        let mut msg = GpsMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__msg__GpsMessage__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GpsMessageSeq<N> {}
unsafe impl<const N: usize> Sync for GpsMessageSeq<N> {}
