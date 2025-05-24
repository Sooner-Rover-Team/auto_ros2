use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct WayPoint {
    pub id: unique_identifier_msgs::msg::UUID,
    pub position: crate::msg::GeoPoint,
    pub props: crate::msg::KeyValueSeq<0>,
}

extern "C" {
    fn geographic_msgs__msg__WayPoint__init(msg: *mut WayPoint) -> bool;
    fn geographic_msgs__msg__WayPoint__fini(msg: *mut WayPoint);
    fn geographic_msgs__msg__WayPoint__are_equal(lhs: *const WayPoint, rhs: *const WayPoint) -> bool;
    fn geographic_msgs__msg__WayPoint__Sequence__init(msg: *mut WayPointSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__WayPoint__Sequence__fini(msg: *mut WayPointSeqRaw);
    fn geographic_msgs__msg__WayPoint__Sequence__are_equal(lhs: *const WayPointSeqRaw, rhs: *const WayPointSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__WayPoint() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for WayPoint {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__WayPoint()
        }
    }
}

impl PartialEq for WayPoint {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__WayPoint__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for WayPointSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = WayPointSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = WayPointSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__WayPoint__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl WayPoint {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__WayPoint__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for WayPoint {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__WayPoint__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct WayPointSeqRaw {
    data: *mut WayPoint,
    size: size_t,
    capacity: size_t,
}

/// Sequence of WayPoint.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct WayPointSeq<const N: usize> {
    data: *mut WayPoint,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> WayPointSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: WayPointSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__WayPoint__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: WayPointSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[WayPoint] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [WayPoint] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, WayPoint> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, WayPoint> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for WayPointSeq<N> {
    fn drop(&mut self) {
        let mut msg = WayPointSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__WayPoint__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for WayPointSeq<N> {}
unsafe impl<const N: usize> Sync for WayPointSeq<N> {}
