use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct RoutePath {
    pub header: std_msgs::msg::Header,
    pub network: unique_identifier_msgs::msg::UUID,
    pub segments: unique_identifier_msgs::msg::UUIDSeq<0>,
    pub props: crate::msg::KeyValueSeq<0>,
}

extern "C" {
    fn geographic_msgs__msg__RoutePath__init(msg: *mut RoutePath) -> bool;
    fn geographic_msgs__msg__RoutePath__fini(msg: *mut RoutePath);
    fn geographic_msgs__msg__RoutePath__are_equal(lhs: *const RoutePath, rhs: *const RoutePath) -> bool;
    fn geographic_msgs__msg__RoutePath__Sequence__init(msg: *mut RoutePathSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__RoutePath__Sequence__fini(msg: *mut RoutePathSeqRaw);
    fn geographic_msgs__msg__RoutePath__Sequence__are_equal(lhs: *const RoutePathSeqRaw, rhs: *const RoutePathSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RoutePath() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for RoutePath {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RoutePath()
        }
    }
}

impl PartialEq for RoutePath {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__RoutePath__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for RoutePathSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = RoutePathSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = RoutePathSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__RoutePath__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl RoutePath {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__RoutePath__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for RoutePath {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__RoutePath__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct RoutePathSeqRaw {
    data: *mut RoutePath,
    size: size_t,
    capacity: size_t,
}

/// Sequence of RoutePath.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct RoutePathSeq<const N: usize> {
    data: *mut RoutePath,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> RoutePathSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: RoutePathSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__RoutePath__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: RoutePathSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[RoutePath] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [RoutePath] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, RoutePath> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, RoutePath> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for RoutePathSeq<N> {
    fn drop(&mut self) {
        let mut msg = RoutePathSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__RoutePath__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for RoutePathSeq<N> {}
unsafe impl<const N: usize> Sync for RoutePathSeq<N> {}
