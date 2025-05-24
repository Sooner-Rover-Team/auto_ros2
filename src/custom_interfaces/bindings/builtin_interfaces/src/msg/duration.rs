use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Duration {
    pub sec: i32,
    pub nanosec: u32,
}

extern "C" {
    fn builtin_interfaces__msg__Duration__init(msg: *mut Duration) -> bool;
    fn builtin_interfaces__msg__Duration__fini(msg: *mut Duration);
    fn builtin_interfaces__msg__Duration__are_equal(lhs: *const Duration, rhs: *const Duration) -> bool;
    fn builtin_interfaces__msg__Duration__Sequence__init(msg: *mut DurationSeqRaw, size: usize) -> bool;
    fn builtin_interfaces__msg__Duration__Sequence__fini(msg: *mut DurationSeqRaw);
    fn builtin_interfaces__msg__Duration__Sequence__are_equal(lhs: *const DurationSeqRaw, rhs: *const DurationSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__builtin_interfaces__msg__Duration() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Duration {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__builtin_interfaces__msg__Duration()
        }
    }
}

impl PartialEq for Duration {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            builtin_interfaces__msg__Duration__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for DurationSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = DurationSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = DurationSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            builtin_interfaces__msg__Duration__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Duration {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { builtin_interfaces__msg__Duration__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Duration {
    fn drop(&mut self) {
        unsafe { builtin_interfaces__msg__Duration__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct DurationSeqRaw {
    data: *mut Duration,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Duration.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct DurationSeq<const N: usize> {
    data: *mut Duration,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> DurationSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: DurationSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { builtin_interfaces__msg__Duration__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: DurationSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Duration] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Duration] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Duration> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Duration> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for DurationSeq<N> {
    fn drop(&mut self) {
        let mut msg = DurationSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { builtin_interfaces__msg__Duration__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for DurationSeq<N> {}
unsafe impl<const N: usize> Sync for DurationSeq<N> {}
