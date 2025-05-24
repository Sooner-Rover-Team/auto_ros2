use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct RouteNetwork {
    pub header: std_msgs::msg::Header,
    pub id: unique_identifier_msgs::msg::UUID,
    pub bounds: crate::msg::BoundingBox,
    pub points: crate::msg::WayPointSeq<0>,
    pub segments: crate::msg::RouteSegmentSeq<0>,
    pub props: crate::msg::KeyValueSeq<0>,
}

extern "C" {
    fn geographic_msgs__msg__RouteNetwork__init(msg: *mut RouteNetwork) -> bool;
    fn geographic_msgs__msg__RouteNetwork__fini(msg: *mut RouteNetwork);
    fn geographic_msgs__msg__RouteNetwork__are_equal(lhs: *const RouteNetwork, rhs: *const RouteNetwork) -> bool;
    fn geographic_msgs__msg__RouteNetwork__Sequence__init(msg: *mut RouteNetworkSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__RouteNetwork__Sequence__fini(msg: *mut RouteNetworkSeqRaw);
    fn geographic_msgs__msg__RouteNetwork__Sequence__are_equal(lhs: *const RouteNetworkSeqRaw, rhs: *const RouteNetworkSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteNetwork() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for RouteNetwork {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteNetwork()
        }
    }
}

impl PartialEq for RouteNetwork {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__RouteNetwork__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for RouteNetworkSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = RouteNetworkSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = RouteNetworkSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__RouteNetwork__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl RouteNetwork {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__RouteNetwork__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for RouteNetwork {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__RouteNetwork__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct RouteNetworkSeqRaw {
    data: *mut RouteNetwork,
    size: size_t,
    capacity: size_t,
}

/// Sequence of RouteNetwork.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct RouteNetworkSeq<const N: usize> {
    data: *mut RouteNetwork,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> RouteNetworkSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: RouteNetworkSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__RouteNetwork__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: RouteNetworkSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[RouteNetwork] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [RouteNetwork] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, RouteNetwork> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, RouteNetwork> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for RouteNetworkSeq<N> {
    fn drop(&mut self) {
        let mut msg = RouteNetworkSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__RouteNetwork__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for RouteNetworkSeq<N> {}
unsafe impl<const N: usize> Sync for RouteNetworkSeq<N> {}
