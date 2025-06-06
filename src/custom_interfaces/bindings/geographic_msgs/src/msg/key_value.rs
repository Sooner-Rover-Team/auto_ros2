use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct KeyValue {
    pub key: safe_drive::msg::RosString<0>,
    pub value: safe_drive::msg::RosString<0>,
}

extern "C" {
    fn geographic_msgs__msg__KeyValue__init(msg: *mut KeyValue) -> bool;
    fn geographic_msgs__msg__KeyValue__fini(msg: *mut KeyValue);
    fn geographic_msgs__msg__KeyValue__are_equal(lhs: *const KeyValue, rhs: *const KeyValue) -> bool;
    fn geographic_msgs__msg__KeyValue__Sequence__init(msg: *mut KeyValueSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__KeyValue__Sequence__fini(msg: *mut KeyValueSeqRaw);
    fn geographic_msgs__msg__KeyValue__Sequence__are_equal(lhs: *const KeyValueSeqRaw, rhs: *const KeyValueSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__KeyValue() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for KeyValue {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__KeyValue()
        }
    }
}

impl PartialEq for KeyValue {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__KeyValue__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for KeyValueSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = KeyValueSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = KeyValueSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__KeyValue__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl KeyValue {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__KeyValue__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for KeyValue {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__KeyValue__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct KeyValueSeqRaw {
    data: *mut KeyValue,
    size: size_t,
    capacity: size_t,
}

/// Sequence of KeyValue.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct KeyValueSeq<const N: usize> {
    data: *mut KeyValue,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> KeyValueSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: KeyValueSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__KeyValue__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: KeyValueSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[KeyValue] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [KeyValue] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, KeyValue> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, KeyValue> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for KeyValueSeq<N> {
    fn drop(&mut self) {
        let mut msg = KeyValueSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__KeyValue__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for KeyValueSeq<N> {}
unsafe impl<const N: usize> Sync for KeyValueSeq<N> {}
