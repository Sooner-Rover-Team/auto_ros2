use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct BatteryMessage {
    pub structure_needs_at_least_one_member: u8,
}

extern "C" {
    fn custom_interfaces__msg__BatteryMessage__init(msg: *mut BatteryMessage) -> bool;
    fn custom_interfaces__msg__BatteryMessage__fini(msg: *mut BatteryMessage);
    fn custom_interfaces__msg__BatteryMessage__are_equal(lhs: *const BatteryMessage, rhs: *const BatteryMessage) -> bool;
    fn custom_interfaces__msg__BatteryMessage__Sequence__init(msg: *mut BatteryMessageSeqRaw, size: usize) -> bool;
    fn custom_interfaces__msg__BatteryMessage__Sequence__fini(msg: *mut BatteryMessageSeqRaw);
    fn custom_interfaces__msg__BatteryMessage__Sequence__are_equal(lhs: *const BatteryMessageSeqRaw, rhs: *const BatteryMessageSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__BatteryMessage() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for BatteryMessage {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__msg__BatteryMessage()
        }
    }
}

impl PartialEq for BatteryMessage {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__msg__BatteryMessage__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for BatteryMessageSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = BatteryMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = BatteryMessageSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__msg__BatteryMessage__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl BatteryMessage {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__BatteryMessage__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for BatteryMessage {
    fn drop(&mut self) {
        unsafe { custom_interfaces__msg__BatteryMessage__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct BatteryMessageSeqRaw {
    data: *mut BatteryMessage,
    size: size_t,
    capacity: size_t,
}

/// Sequence of BatteryMessage.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct BatteryMessageSeq<const N: usize> {
    data: *mut BatteryMessage,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> BatteryMessageSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: BatteryMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__msg__BatteryMessage__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: BatteryMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[BatteryMessage] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [BatteryMessage] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, BatteryMessage> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, BatteryMessage> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for BatteryMessageSeq<N> {
    fn drop(&mut self) {
        let mut msg = BatteryMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__msg__BatteryMessage__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for BatteryMessageSeq<N> {}
unsafe impl<const N: usize> Sync for BatteryMessageSeq<N> {}
