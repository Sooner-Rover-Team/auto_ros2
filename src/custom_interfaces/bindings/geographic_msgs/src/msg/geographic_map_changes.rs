use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct GeographicMapChanges {
    pub header: std_msgs::msg::Header,
    pub diffs: crate::msg::GeographicMap,
    pub deletes: unique_identifier_msgs::msg::UUIDSeq<0>,
}

extern "C" {
    fn geographic_msgs__msg__GeographicMapChanges__init(msg: *mut GeographicMapChanges) -> bool;
    fn geographic_msgs__msg__GeographicMapChanges__fini(msg: *mut GeographicMapChanges);
    fn geographic_msgs__msg__GeographicMapChanges__are_equal(lhs: *const GeographicMapChanges, rhs: *const GeographicMapChanges) -> bool;
    fn geographic_msgs__msg__GeographicMapChanges__Sequence__init(msg: *mut GeographicMapChangesSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__GeographicMapChanges__Sequence__fini(msg: *mut GeographicMapChangesSeqRaw);
    fn geographic_msgs__msg__GeographicMapChanges__Sequence__are_equal(lhs: *const GeographicMapChangesSeqRaw, rhs: *const GeographicMapChangesSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMapChanges() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GeographicMapChanges {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMapChanges()
        }
    }
}

impl PartialEq for GeographicMapChanges {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__GeographicMapChanges__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GeographicMapChangesSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GeographicMapChangesSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GeographicMapChangesSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__GeographicMapChanges__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GeographicMapChanges {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeographicMapChanges__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GeographicMapChanges {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__GeographicMapChanges__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GeographicMapChangesSeqRaw {
    data: *mut GeographicMapChanges,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GeographicMapChanges.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GeographicMapChangesSeq<const N: usize> {
    data: *mut GeographicMapChanges,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GeographicMapChangesSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GeographicMapChangesSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeographicMapChanges__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GeographicMapChangesSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GeographicMapChanges] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GeographicMapChanges] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GeographicMapChanges> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GeographicMapChanges> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GeographicMapChangesSeq<N> {
    fn drop(&mut self) {
        let mut msg = GeographicMapChangesSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__GeographicMapChanges__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GeographicMapChangesSeq<N> {}
unsafe impl<const N: usize> Sync for GeographicMapChangesSeq<N> {}
