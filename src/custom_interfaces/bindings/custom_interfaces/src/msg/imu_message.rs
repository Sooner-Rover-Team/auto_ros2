use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct ImuMessage {
    pub accel: geometry_msgs::msg::Vector3,
    pub gyro: geometry_msgs::msg::Vector3,
    pub compass: geometry_msgs::msg::Vector3,
    pub temp_c: f64,
}

extern "C" {
    fn custom_interfaces__msg__ImuMessage__init(msg: *mut ImuMessage) -> bool;
    fn custom_interfaces__msg__ImuMessage__fini(msg: *mut ImuMessage);
    fn custom_interfaces__msg__ImuMessage__are_equal(lhs: *const ImuMessage, rhs: *const ImuMessage) -> bool;
    fn custom_interfaces__msg__ImuMessage__Sequence__init(msg: *mut ImuMessageSeqRaw, size: usize) -> bool;
    fn custom_interfaces__msg__ImuMessage__Sequence__fini(msg: *mut ImuMessageSeqRaw);
    fn custom_interfaces__msg__ImuMessage__Sequence__are_equal(lhs: *const ImuMessageSeqRaw, rhs: *const ImuMessageSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__ImuMessage() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for ImuMessage {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__ImuMessage()
        }
    }
}

impl PartialEq for ImuMessage {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__msg__ImuMessage__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for ImuMessageSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = ImuMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = ImuMessageSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__msg__ImuMessage__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl ImuMessage {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__ImuMessage__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for ImuMessage {
    fn drop(&mut self) {
        unsafe { custom_interfaces__msg__ImuMessage__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct ImuMessageSeqRaw {
    data: *mut ImuMessage,
    size: size_t,
    capacity: size_t,
}

/// Sequence of ImuMessage.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct ImuMessageSeq<const N: usize> {
    data: *mut ImuMessage,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> ImuMessageSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: ImuMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__ImuMessage__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: ImuMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[ImuMessage] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [ImuMessage] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, ImuMessage> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, ImuMessage> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for ImuMessageSeq<N> {
    fn drop(&mut self) {
        let mut msg = ImuMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__msg__ImuMessage__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for ImuMessageSeq<N> {}
unsafe impl<const N: usize> Sync for ImuMessageSeq<N> {}
