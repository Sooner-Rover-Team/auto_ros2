use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct WheelsMessage {
    pub left_wheels: u8,
    pub right_wheels: u8,
}

extern "C" {
    fn custom_interfaces__msg__WheelsMessage__init(msg: *mut WheelsMessage) -> bool;
    fn custom_interfaces__msg__WheelsMessage__fini(msg: *mut WheelsMessage);
    fn custom_interfaces__msg__WheelsMessage__are_equal(lhs: *const WheelsMessage, rhs: *const WheelsMessage) -> bool;
    fn custom_interfaces__msg__WheelsMessage__Sequence__init(msg: *mut WheelsMessageSeqRaw, size: usize) -> bool;
    fn custom_interfaces__msg__WheelsMessage__Sequence__fini(msg: *mut WheelsMessageSeqRaw);
    fn custom_interfaces__msg__WheelsMessage__Sequence__are_equal(lhs: *const WheelsMessageSeqRaw, rhs: *const WheelsMessageSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__WheelsMessage() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for WheelsMessage {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__WheelsMessage()
        }
    }
}

impl PartialEq for WheelsMessage {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__msg__WheelsMessage__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for WheelsMessageSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = WheelsMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = WheelsMessageSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__msg__WheelsMessage__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl WheelsMessage {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__WheelsMessage__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for WheelsMessage {
    fn drop(&mut self) {
        unsafe { custom_interfaces__msg__WheelsMessage__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct WheelsMessageSeqRaw {
    data: *mut WheelsMessage,
    size: size_t,
    capacity: size_t,
}

/// Sequence of WheelsMessage.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct WheelsMessageSeq<const N: usize> {
    data: *mut WheelsMessage,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> WheelsMessageSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: WheelsMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__WheelsMessage__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: WheelsMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[WheelsMessage] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [WheelsMessage] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, WheelsMessage> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, WheelsMessage> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for WheelsMessageSeq<N> {
    fn drop(&mut self) {
        let mut msg = WheelsMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__msg__WheelsMessage__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for WheelsMessageSeq<N> {}
unsafe impl<const N: usize> Sync for WheelsMessageSeq<N> {}
