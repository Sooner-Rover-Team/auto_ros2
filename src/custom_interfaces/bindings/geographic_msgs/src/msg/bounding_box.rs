use safe_drive::{msg::TypeSupport, rcl::{self, size_t}};

#[repr(C)]
#[derive(Debug)]
pub struct BoundingBox {
    pub min_pt: crate::msg::GeoPoint,
    pub max_pt: crate::msg::GeoPoint,
}

extern "C" {
    fn geographic_msgs__msg__BoundingBox__init(msg: *mut BoundingBox) -> bool;
    fn geographic_msgs__msg__BoundingBox__fini(msg: *mut BoundingBox);
    fn geographic_msgs__msg__BoundingBox__are_equal(lhs: *const BoundingBox, rhs: *const BoundingBox) -> bool;
    fn geographic_msgs__msg__BoundingBox__Sequence__init(msg: *mut BoundingBoxSeqRaw, size: usize) -> bool;
    fn geographic_msgs__msg__BoundingBox__Sequence__fini(msg: *mut BoundingBoxSeqRaw);
    fn geographic_msgs__msg__BoundingBox__Sequence__are_equal(lhs: *const BoundingBoxSeqRaw, rhs: *const BoundingBoxSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__BoundingBox() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for BoundingBox {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__BoundingBox()
        }
    }
}

impl PartialEq for BoundingBox {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            geographic_msgs__msg__BoundingBox__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for BoundingBoxSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = BoundingBoxSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = BoundingBoxSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            geographic_msgs__msg__BoundingBox__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl BoundingBox {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__BoundingBox__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for BoundingBox {
    fn drop(&mut self) {
        unsafe { geographic_msgs__msg__BoundingBox__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct BoundingBoxSeqRaw {
    data: *mut BoundingBox,
    size: size_t,
    capacity: size_t,
}

/// Sequence of BoundingBox.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct BoundingBoxSeq<const N: usize> {
    data: *mut BoundingBox,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> BoundingBoxSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: BoundingBoxSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { geographic_msgs__msg__BoundingBox__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: BoundingBoxSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[BoundingBox] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [BoundingBox] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, BoundingBox> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, BoundingBox> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for BoundingBoxSeq<N> {
    fn drop(&mut self) {
        let mut msg = BoundingBoxSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { geographic_msgs__msg__BoundingBox__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for BoundingBoxSeq<N> {}
unsafe impl<const N: usize> Sync for BoundingBoxSeq<N> {}
