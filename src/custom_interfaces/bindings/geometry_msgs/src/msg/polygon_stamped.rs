use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct PolygonStamped {
    pub header: std_msgs::msg::Header,
    pub polygon: crate::msg::Polygon,
}

extern "C" {
    fn geometry_msgs__msg__PolygonStamped__init(msg: *mut PolygonStamped) -> bool;
    fn geometry_msgs__msg__PolygonStamped__fini(msg: *mut PolygonStamped);
    fn geometry_msgs__msg__PolygonStamped__are_equal(lhs: *const PolygonStamped, rhs: *const PolygonStamped) -> bool;
    fn geometry_msgs__msg__PolygonStamped__Sequence__init(msg: *mut PolygonStampedSeqRaw, size: usize) -> bool;
    fn geometry_msgs__msg__PolygonStamped__Sequence__fini(msg: *mut PolygonStampedSeqRaw);
    fn geometry_msgs__msg__PolygonStamped__Sequence__are_equal(lhs: *const PolygonStampedSeqRaw, rhs: *const PolygonStampedSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PolygonStamped() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for PolygonStamped {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geometry_msgs__msg__PolygonStamped()
        }
    }
}

impl PartialEq for PolygonStamped {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geometry_msgs__msg__PolygonStamped__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for PolygonStampedSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = PolygonStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = PolygonStampedSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geometry_msgs__msg__PolygonStamped__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl PolygonStamped {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__PolygonStamped__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for PolygonStamped {
    fn drop(&mut self) {
        unsafe { geometry_msgs__msg__PolygonStamped__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct PolygonStampedSeqRaw {
    data: *mut PolygonStamped,
    size: size_t,
    capacity: size_t,
}

/// Sequence of PolygonStamped.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct PolygonStampedSeq<const N: usize> {
    data: *mut PolygonStamped,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> PolygonStampedSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: PolygonStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geometry_msgs__msg__PolygonStamped__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: PolygonStampedSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[PolygonStamped] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [PolygonStamped] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, PolygonStamped> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, PolygonStamped> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for PolygonStampedSeq<N> {
    fn drop(&mut self) {
        let mut msg = PolygonStampedSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geometry_msgs__msg__PolygonStamped__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for PolygonStampedSeq<N> {}
unsafe impl<const N: usize> Sync for PolygonStampedSeq<N> {}
