use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct RouteSegment {
    pub id: unique_identifier_msgs::msg::UUID,
    pub start: unique_identifier_msgs::msg::UUID,
    pub end: unique_identifier_msgs::msg::UUID,
    pub props: crate::msg::KeyValueSeq<0>,
}

extern "C" {
    fn geographic_msgs__msg__RouteSegment__init(msg: *mut RouteSegment) -> bool;
    fn geographic_msgs__msg__RouteSegment__fini(msg: *mut RouteSegment);
    fn geographic_msgs__msg__RouteSegment__are_equal(lhs: *const RouteSegment, rhs: *const RouteSegment) -> bool;
    fn geographic_msgs__msg__RouteSegment__Sequence__init(msg: *mut RouteSegmentSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__RouteSegment__Sequence__fini(msg: *mut RouteSegmentSeqRaw);
    fn geographic_msgs__msg__RouteSegment__Sequence__are_equal(lhs: *const RouteSegmentSeqRaw, rhs: *const RouteSegmentSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteSegment() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for RouteSegment {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteSegment()
        }
    }
}

impl PartialEq for RouteSegment {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__RouteSegment__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for RouteSegmentSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = RouteSegmentSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = RouteSegmentSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__RouteSegment__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl RouteSegment {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__RouteSegment__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for RouteSegment {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__RouteSegment__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct RouteSegmentSeqRaw {
    data: *mut RouteSegment,
    size: size_t,
    capacity: size_t,
}

/// Sequence of RouteSegment.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct RouteSegmentSeq<const N: usize> {
    data: *mut RouteSegment,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> RouteSegmentSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: RouteSegmentSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__RouteSegment__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: RouteSegmentSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[RouteSegment] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [RouteSegment] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, RouteSegment> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, RouteSegment> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for RouteSegmentSeq<N> {
    fn drop(&mut self) {
        let mut msg = RouteSegmentSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__RouteSegment__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for RouteSegmentSeq<N> {}
unsafe impl<const N: usize> Sync for RouteSegmentSeq<N> {}
