use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

type uint8__16 = [u8; 16];

#[repr(C)]
#[derive(Debug)]
pub struct UUID {
    pub uuid: uint8__16,
}

extern "C" {
    fn unique_identifier_msgs__msg__UUID__init(msg: *mut UUID) -> bool;
    fn unique_identifier_msgs__msg__UUID__fini(msg: *mut UUID);
    fn unique_identifier_msgs__msg__UUID__are_equal(lhs: *const UUID, rhs: *const UUID) -> bool;
    fn unique_identifier_msgs__msg__UUID__Sequence__init(msg: *mut UUIDSeqRaw, size: usize) -> bool;
    fn unique_identifier_msgs__msg__UUID__Sequence__fini(msg: *mut UUIDSeqRaw);
    fn unique_identifier_msgs__msg__UUID__Sequence__are_equal(lhs: *const UUIDSeqRaw, rhs: *const UUIDSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__unique_identifier_msgs__msg__UUID() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for UUID {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__unique_identifier_msgs__msg__UUID()
        }
    }
}

impl PartialEq for UUID {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            unique_identifier_msgs__msg__UUID__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for UUIDSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = UUIDSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = UUIDSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            unique_identifier_msgs__msg__UUID__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl UUID {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { unique_identifier_msgs__msg__UUID__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for UUID {
    fn drop(&mut self) {
        unsafe { unique_identifier_msgs__msg__UUID__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct UUIDSeqRaw {
    data: *mut UUID,
    size: size_t,
    capacity: size_t,
}

/// Sequence of UUID.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct UUIDSeq<const N: usize> {
    data: *mut UUID,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> UUIDSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: UUIDSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { unique_identifier_msgs__msg__UUID__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: UUIDSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[UUID] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [UUID] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, UUID> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, UUID> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for UUIDSeq<N> {
    fn drop(&mut self) {
        let mut msg = UUIDSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { unique_identifier_msgs__msg__UUID__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for UUIDSeq<N> {}
unsafe impl<const N: usize> Sync for UUIDSeq<N> {}
