use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct Int32 {
    pub data: i32,
}

extern "C" {
    fn std_msgs__msg__Int32__init(msg: *mut Int32) -> bool;
    fn std_msgs__msg__Int32__fini(msg: *mut Int32);
    fn std_msgs__msg__Int32__are_equal(lhs: *const Int32, rhs: *const Int32) -> bool;
    fn std_msgs__msg__Int32__Sequence__init(msg: *mut Int32SeqRaw, size: usize) -> bool;
    fn std_msgs__msg__Int32__Sequence__fini(msg: *mut Int32SeqRaw);
    fn std_msgs__msg__Int32__Sequence__are_equal(lhs: *const Int32SeqRaw, rhs: *const Int32SeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for Int32 {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32()
        }
    }
}

impl PartialEq for Int32 {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__Int32__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for Int32Seq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = Int32SeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = Int32SeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__Int32__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl Int32 {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Int32__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for Int32 {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__Int32__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct Int32SeqRaw {
    data: *mut Int32,
    size: size_t,
    capacity: size_t,
}

/// Sequence of Int32.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct Int32Seq<const N: usize> {
    data: *mut Int32,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> Int32Seq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: Int32SeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__Int32__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: Int32SeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[Int32] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [Int32] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Int32> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Int32> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for Int32Seq<N> {
    fn drop(&mut self) {
        let mut msg = Int32SeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__Int32__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for Int32Seq<N> {}
unsafe impl<const N: usize> Sync for Int32Seq<N> {}
