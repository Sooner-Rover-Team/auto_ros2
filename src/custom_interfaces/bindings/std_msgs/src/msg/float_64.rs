use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Float64 {
    pub data: f64,
}

extern "C" {
    fn std_msgs__msg__Float64__init(msg: *mut Float64) -> bool;
    fn std_msgs__msg__Float64__fini(msg: *mut Float64);
    fn std_msgs__msg__Float64__are_equal(lhs: *const Float64, rhs: *const Float64) -> bool;
    fn std_msgs__msg__Float64__Sequence__init(msg: *mut Float64SeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Float64__Sequence__fini(msg: *mut Float64SeqRaw);
    fn std_msgs__msg__Float64__Sequence__are_equal(lhs: *const Float64SeqRaw, rhs: *const Float64SeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Float64() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Float64 {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Float64()
        }
    }
}

impl PartialEq for Float64 {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Float64__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Float64Seq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Float64SeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Float64SeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Float64__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Float64 {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Float64__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Float64 {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Float64__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Float64SeqRaw {
    data: *mut Float64,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Float64.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Float64Seq<const N: usize> {
    data: *mut Float64,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Float64Seq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Float64SeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Float64__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Float64SeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Float64] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Float64] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Float64> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Float64> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Float64Seq<N> {
    fn drop(&mut self) {
        let mut msg = Float64SeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Float64__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Float64Seq<N> {}
unsafe impl<const N: usize> Sync for Float64Seq<N> {}
