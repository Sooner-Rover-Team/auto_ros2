use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct ColorRGBA {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

extern "C" {
    fn std_msgs__msg__ColorRGBA__init(msg: *mut ColorRGBA) -> bool;
    fn std_msgs__msg__ColorRGBA__fini(msg: *mut ColorRGBA);
    fn std_msgs__msg__ColorRGBA__are_equal(lhs: *const ColorRGBA, rhs: *const ColorRGBA) -> bool;
    fn std_msgs__msg__ColorRGBA__Sequence__init(msg: *mut ColorRGBASeqRaw, size: usize) -> bool;
    fn std_msgs__msg__ColorRGBA__Sequence__fini(msg: *mut ColorRGBASeqRaw);
    fn std_msgs__msg__ColorRGBA__Sequence__are_equal(lhs: *const ColorRGBASeqRaw, rhs: *const ColorRGBASeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__ColorRGBA() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for ColorRGBA {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__ColorRGBA()
        }
    }
}

impl PartialEq for ColorRGBA {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            std_msgs__msg__ColorRGBA__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for ColorRGBASeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = ColorRGBASeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = ColorRGBASeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            std_msgs__msg__ColorRGBA__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl ColorRGBA {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__ColorRGBA__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for ColorRGBA {
    fn drop(&mut self) {
        unsafe { std_msgs__msg__ColorRGBA__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct ColorRGBASeqRaw {
    data: *mut ColorRGBA,
    size: size_t,
    capacity: size_t,
}

/// Sequence of ColorRGBA.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct ColorRGBASeq<const N: usize> {
    data: *mut ColorRGBA,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> ColorRGBASeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: ColorRGBASeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { std_msgs__msg__ColorRGBA__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: ColorRGBASeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[ColorRGBA] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [ColorRGBA] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, ColorRGBA> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, ColorRGBA> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for ColorRGBASeq<N> {
    fn drop(&mut self) {
        let mut msg = ColorRGBASeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { std_msgs__msg__ColorRGBA__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for ColorRGBASeq<N> {}
unsafe impl<const N: usize> Sync for ColorRGBASeq<N> {}
