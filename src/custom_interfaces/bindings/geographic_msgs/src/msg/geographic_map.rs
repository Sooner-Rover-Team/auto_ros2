use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct GeographicMap {
    pub header: std_msgs::msg::Header,
    pub id: unique_identifier_msgs::msg::UUID,
    pub bounds: crate::msg::BoundingBox,
    pub points: crate::msg::WayPointSeq<0>,
    pub features: crate::msg::MapFeatureSeq<0>,
    pub props: crate::msg::KeyValueSeq<0>,
}

extern "C" {
    fn geographic_msgs__msg__GeographicMap__init(msg: *mut GeographicMap) -> bool;
    fn geographic_msgs__msg__GeographicMap__fini(msg: *mut GeographicMap);
    fn geographic_msgs__msg__GeographicMap__are_equal(lhs: *const GeographicMap, rhs: *const GeographicMap) -> bool;
    fn geographic_msgs__msg__GeographicMap__Sequence__init(msg: *mut GeographicMapSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__GeographicMap__Sequence__fini(msg: *mut GeographicMapSeqRaw);
    fn geographic_msgs__msg__GeographicMap__Sequence__are_equal(lhs: *const GeographicMapSeqRaw, rhs: *const GeographicMapSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMap() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for GeographicMap {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMap()
        }
    }
}

impl PartialEq for GeographicMap {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__GeographicMap__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for GeographicMapSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = GeographicMapSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = GeographicMapSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__GeographicMap__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl GeographicMap {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeographicMap__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for GeographicMap {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__GeographicMap__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct GeographicMapSeqRaw {
    data: *mut GeographicMap,
    size: size_t,
    capacity: size_t,
}

/// Sequence of GeographicMap.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct GeographicMapSeq<const N: usize> {
    data: *mut GeographicMap,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> GeographicMapSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: GeographicMapSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__GeographicMap__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: GeographicMapSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[GeographicMap] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [GeographicMap] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, GeographicMap> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, GeographicMap> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for GeographicMapSeq<N> {
    fn drop(&mut self) {
        let mut msg = GeographicMapSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__GeographicMap__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for GeographicMapSeq<N> {}
unsafe impl<const N: usize> Sync for GeographicMapSeq<N> {}
