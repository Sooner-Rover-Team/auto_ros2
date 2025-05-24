use safe_drive::{msg::{ActionMsg, ActionGoal, ActionResult, GetUUID, GoalResponse, ResultResponse, TypeSupport, builtin_interfaces::UnsafeTime, unique_identifier_msgs}, rcl::{self, size_t}};

extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__custom_interfaces__action__TrackArucoMarker() -> *const rcl::rosidl_action_type_support_t;
}

#[derive(Debug)]
pub struct TrackArucoMarker;

impl ActionMsg for TrackArucoMarker {
    type Goal = TrackArucoMarker_SendGoal;
    type Result = TrackArucoMarker_GetResult;
    type Feedback = TrackArucoMarker_FeedbackMessage;
    fn type_support() -> *const rcl::rosidl_action_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_action_type_support_handle__custom_interfaces__action__TrackArucoMarker()
        }
    }

    type GoalContent = TrackArucoMarker_Goal;

    fn new_goal_request(
        goal: Self::GoalContent,
        uuid: [u8; 16],
    ) -> <Self::Goal as ActionGoal>::Request {
        TrackArucoMarker_SendGoal_Request {
            goal,
            goal_id: unique_identifier_msgs::msg::UUID { uuid },
        }
    }

    type ResultContent = TrackArucoMarker_Result;

    fn new_result_response(
        status: u8,
        result: Self::ResultContent,
    ) -> <Self::Result as ActionResult>::Response {
        TrackArucoMarker_GetResult_Response { status, result }
    }

    type FeedbackContent = TrackArucoMarker_Feedback;

    fn new_feedback_message(feedback: Self::FeedbackContent, uuid: [u8; 16]) -> Self::Feedback {
        TrackArucoMarker_FeedbackMessage {
            feedback,
            goal_id: unique_identifier_msgs::msg::UUID { uuid },
        }
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: TrackArucoMarker_Goal,
}

#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_SendGoal_Response {
    pub accepted: bool,
    pub stamp: UnsafeTime,
}

#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}

#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_GetResult_Response {
    pub status: u8,
    pub result: TrackArucoMarker_Result,
}

#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: TrackArucoMarker_Feedback,
}

#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_Goal {
    pub marker_id: u8,
}

extern "C" {
    fn custom_interfaces__action__TrackArucoMarker_Goal__init(msg: *mut TrackArucoMarker_Goal) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Goal__fini(msg: *mut TrackArucoMarker_Goal);
    fn custom_interfaces__action__TrackArucoMarker_Goal__are_equal(lhs: *const TrackArucoMarker_Goal, rhs: *const TrackArucoMarker_Goal) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Goal__Sequence__init(msg: *mut TrackArucoMarker_GoalSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Goal__Sequence__fini(msg: *mut TrackArucoMarker_GoalSeqRaw);
    fn custom_interfaces__action__TrackArucoMarker_Goal__Sequence__are_equal(lhs: *const TrackArucoMarker_GoalSeqRaw, rhs: *const TrackArucoMarker_GoalSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_Goal() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TrackArucoMarker_Goal {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_Goal()
        }
    }
}

impl PartialEq for TrackArucoMarker_Goal {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__TrackArucoMarker_Goal__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TrackArucoMarker_GoalSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TrackArucoMarker_GoalSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TrackArucoMarker_GoalSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__TrackArucoMarker_Goal__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TrackArucoMarker_Goal {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_Goal__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TrackArucoMarker_Goal {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__TrackArucoMarker_Goal__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TrackArucoMarker_GoalSeqRaw {
    data: *mut TrackArucoMarker_Goal,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TrackArucoMarker_Goal.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_GoalSeq<const N: usize> {
    data: *mut TrackArucoMarker_Goal,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TrackArucoMarker_GoalSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TrackArucoMarker_GoalSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_Goal__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TrackArucoMarker_GoalSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TrackArucoMarker_Goal] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TrackArucoMarker_Goal] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TrackArucoMarker_Goal> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TrackArucoMarker_Goal> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TrackArucoMarker_GoalSeq<N> {
    fn drop(&mut self) {
        let mut msg = TrackArucoMarker_GoalSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__TrackArucoMarker_Goal__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TrackArucoMarker_GoalSeq<N> {}
unsafe impl<const N: usize> Sync for TrackArucoMarker_GoalSeq<N> {}

extern "C" {
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Request__init(msg: *mut TrackArucoMarker_SendGoal_Request) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Request__fini(msg: *mut TrackArucoMarker_SendGoal_Request);
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Request__are_equal(lhs: *const TrackArucoMarker_SendGoal_Request, rhs: *const TrackArucoMarker_SendGoal_Request) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Request__Sequence__init(msg: *mut TrackArucoMarker_SendGoal_RequestSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Request__Sequence__fini(msg: *mut TrackArucoMarker_SendGoal_RequestSeqRaw);
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Request__Sequence__are_equal(lhs: *const TrackArucoMarker_SendGoal_RequestSeqRaw, rhs: *const TrackArucoMarker_SendGoal_RequestSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_SendGoal_Request() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TrackArucoMarker_SendGoal_Request {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_SendGoal_Request()
        }
    }
}

impl PartialEq for TrackArucoMarker_SendGoal_Request {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__TrackArucoMarker_SendGoal_Request__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TrackArucoMarker_SendGoal_RequestSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TrackArucoMarker_SendGoal_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TrackArucoMarker_SendGoal_RequestSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__TrackArucoMarker_SendGoal_Request__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TrackArucoMarker_SendGoal_Request {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_SendGoal_Request__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TrackArucoMarker_SendGoal_Request {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__TrackArucoMarker_SendGoal_Request__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TrackArucoMarker_SendGoal_RequestSeqRaw {
    data: *mut TrackArucoMarker_SendGoal_Request,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TrackArucoMarker_SendGoal_Request.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_SendGoal_RequestSeq<const N: usize> {
    data: *mut TrackArucoMarker_SendGoal_Request,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TrackArucoMarker_SendGoal_RequestSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TrackArucoMarker_SendGoal_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_SendGoal_Request__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TrackArucoMarker_SendGoal_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TrackArucoMarker_SendGoal_Request] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TrackArucoMarker_SendGoal_Request] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TrackArucoMarker_SendGoal_Request> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TrackArucoMarker_SendGoal_Request> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TrackArucoMarker_SendGoal_RequestSeq<N> {
    fn drop(&mut self) {
        let mut msg = TrackArucoMarker_SendGoal_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__TrackArucoMarker_SendGoal_Request__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TrackArucoMarker_SendGoal_RequestSeq<N> {}
unsafe impl<const N: usize> Sync for TrackArucoMarker_SendGoal_RequestSeq<N> {}

extern "C" {
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Response__init(msg: *mut TrackArucoMarker_SendGoal_Response) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Response__fini(msg: *mut TrackArucoMarker_SendGoal_Response);
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Response__are_equal(lhs: *const TrackArucoMarker_SendGoal_Response, rhs: *const TrackArucoMarker_SendGoal_Response) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Response__Sequence__init(msg: *mut TrackArucoMarker_SendGoal_ResponseSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Response__Sequence__fini(msg: *mut TrackArucoMarker_SendGoal_ResponseSeqRaw);
    fn custom_interfaces__action__TrackArucoMarker_SendGoal_Response__Sequence__are_equal(lhs: *const TrackArucoMarker_SendGoal_ResponseSeqRaw, rhs: *const TrackArucoMarker_SendGoal_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_SendGoal_Response() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TrackArucoMarker_SendGoal_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_SendGoal_Response()
        }
    }
}

impl PartialEq for TrackArucoMarker_SendGoal_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__TrackArucoMarker_SendGoal_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TrackArucoMarker_SendGoal_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TrackArucoMarker_SendGoal_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TrackArucoMarker_SendGoal_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__TrackArucoMarker_SendGoal_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TrackArucoMarker_SendGoal_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_SendGoal_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TrackArucoMarker_SendGoal_Response {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__TrackArucoMarker_SendGoal_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TrackArucoMarker_SendGoal_ResponseSeqRaw {
    data: *mut TrackArucoMarker_SendGoal_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TrackArucoMarker_SendGoal_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_SendGoal_ResponseSeq<const N: usize> {
    data: *mut TrackArucoMarker_SendGoal_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TrackArucoMarker_SendGoal_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TrackArucoMarker_SendGoal_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_SendGoal_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TrackArucoMarker_SendGoal_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TrackArucoMarker_SendGoal_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TrackArucoMarker_SendGoal_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TrackArucoMarker_SendGoal_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TrackArucoMarker_SendGoal_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TrackArucoMarker_SendGoal_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = TrackArucoMarker_SendGoal_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__TrackArucoMarker_SendGoal_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TrackArucoMarker_SendGoal_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for TrackArucoMarker_SendGoal_ResponseSeq<N> {}

extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__action__TrackArucoMarker_SendGoal() -> *const rcl::rosidl_service_type_support_t;
}

#[derive(Debug)]
pub struct TrackArucoMarker_SendGoal;

impl ActionGoal for TrackArucoMarker_SendGoal {
    type Request = TrackArucoMarker_SendGoal_Request;
    type Response = TrackArucoMarker_SendGoal_Response;
    fn type_support() -> *const rcl::rosidl_service_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__action__TrackArucoMarker_SendGoal()
        }
    }
}

impl GetUUID for TrackArucoMarker_SendGoal_Request {
    fn get_uuid(&self) -> &[u8; 16] {
        &self.goal_id.uuid
    }
}

impl GoalResponse for TrackArucoMarker_SendGoal_Response {
    fn is_accepted(&self) -> bool {
        self.accepted
    }

    fn get_time_stamp(&self) -> UnsafeTime {
        UnsafeTime {
            sec: self.stamp.sec,
            nanosec: self.stamp.nanosec,
        }
    }

    fn new(accepted: bool, stamp: UnsafeTime) -> Self {
        Self { accepted, stamp }
    }
}


#[repr(C)]
#[derive(Clone, Debug)]
pub struct TrackArucoMarker_Result {
    pub structure_needs_at_least_one_member: u8,
}

extern "C" {
    fn custom_interfaces__action__TrackArucoMarker_Result__init(msg: *mut TrackArucoMarker_Result) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Result__fini(msg: *mut TrackArucoMarker_Result);
    fn custom_interfaces__action__TrackArucoMarker_Result__are_equal(lhs: *const TrackArucoMarker_Result, rhs: *const TrackArucoMarker_Result) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Result__Sequence__init(msg: *mut TrackArucoMarker_ResultSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Result__Sequence__fini(msg: *mut TrackArucoMarker_ResultSeqRaw);
    fn custom_interfaces__action__TrackArucoMarker_Result__Sequence__are_equal(lhs: *const TrackArucoMarker_ResultSeqRaw, rhs: *const TrackArucoMarker_ResultSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_Result() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TrackArucoMarker_Result {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_Result()
        }
    }
}

impl PartialEq for TrackArucoMarker_Result {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__TrackArucoMarker_Result__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TrackArucoMarker_ResultSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TrackArucoMarker_ResultSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TrackArucoMarker_ResultSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__TrackArucoMarker_Result__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TrackArucoMarker_Result {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_Result__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TrackArucoMarker_Result {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__TrackArucoMarker_Result__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TrackArucoMarker_ResultSeqRaw {
    data: *mut TrackArucoMarker_Result,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TrackArucoMarker_Result.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_ResultSeq<const N: usize> {
    data: *mut TrackArucoMarker_Result,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TrackArucoMarker_ResultSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TrackArucoMarker_ResultSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_Result__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TrackArucoMarker_ResultSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TrackArucoMarker_Result] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TrackArucoMarker_Result] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TrackArucoMarker_Result> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TrackArucoMarker_Result> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TrackArucoMarker_ResultSeq<N> {
    fn drop(&mut self) {
        let mut msg = TrackArucoMarker_ResultSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__TrackArucoMarker_Result__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TrackArucoMarker_ResultSeq<N> {}
unsafe impl<const N: usize> Sync for TrackArucoMarker_ResultSeq<N> {}

extern "C" {
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Request__init(msg: *mut TrackArucoMarker_GetResult_Request) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Request__fini(msg: *mut TrackArucoMarker_GetResult_Request);
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Request__are_equal(lhs: *const TrackArucoMarker_GetResult_Request, rhs: *const TrackArucoMarker_GetResult_Request) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Request__Sequence__init(msg: *mut TrackArucoMarker_GetResult_RequestSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Request__Sequence__fini(msg: *mut TrackArucoMarker_GetResult_RequestSeqRaw);
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Request__Sequence__are_equal(lhs: *const TrackArucoMarker_GetResult_RequestSeqRaw, rhs: *const TrackArucoMarker_GetResult_RequestSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_GetResult_Request() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TrackArucoMarker_GetResult_Request {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_GetResult_Request()
        }
    }
}

impl PartialEq for TrackArucoMarker_GetResult_Request {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__TrackArucoMarker_GetResult_Request__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TrackArucoMarker_GetResult_RequestSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TrackArucoMarker_GetResult_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TrackArucoMarker_GetResult_RequestSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__TrackArucoMarker_GetResult_Request__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TrackArucoMarker_GetResult_Request {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_GetResult_Request__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TrackArucoMarker_GetResult_Request {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__TrackArucoMarker_GetResult_Request__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TrackArucoMarker_GetResult_RequestSeqRaw {
    data: *mut TrackArucoMarker_GetResult_Request,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TrackArucoMarker_GetResult_Request.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_GetResult_RequestSeq<const N: usize> {
    data: *mut TrackArucoMarker_GetResult_Request,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TrackArucoMarker_GetResult_RequestSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TrackArucoMarker_GetResult_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_GetResult_Request__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TrackArucoMarker_GetResult_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TrackArucoMarker_GetResult_Request] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TrackArucoMarker_GetResult_Request] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TrackArucoMarker_GetResult_Request> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TrackArucoMarker_GetResult_Request> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TrackArucoMarker_GetResult_RequestSeq<N> {
    fn drop(&mut self) {
        let mut msg = TrackArucoMarker_GetResult_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__TrackArucoMarker_GetResult_Request__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TrackArucoMarker_GetResult_RequestSeq<N> {}
unsafe impl<const N: usize> Sync for TrackArucoMarker_GetResult_RequestSeq<N> {}

extern "C" {
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Response__init(msg: *mut TrackArucoMarker_GetResult_Response) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Response__fini(msg: *mut TrackArucoMarker_GetResult_Response);
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Response__are_equal(lhs: *const TrackArucoMarker_GetResult_Response, rhs: *const TrackArucoMarker_GetResult_Response) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Response__Sequence__init(msg: *mut TrackArucoMarker_GetResult_ResponseSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Response__Sequence__fini(msg: *mut TrackArucoMarker_GetResult_ResponseSeqRaw);
    fn custom_interfaces__action__TrackArucoMarker_GetResult_Response__Sequence__are_equal(lhs: *const TrackArucoMarker_GetResult_ResponseSeqRaw, rhs: *const TrackArucoMarker_GetResult_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_GetResult_Response() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TrackArucoMarker_GetResult_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_GetResult_Response()
        }
    }
}

impl PartialEq for TrackArucoMarker_GetResult_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__TrackArucoMarker_GetResult_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TrackArucoMarker_GetResult_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TrackArucoMarker_GetResult_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TrackArucoMarker_GetResult_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__TrackArucoMarker_GetResult_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TrackArucoMarker_GetResult_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_GetResult_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TrackArucoMarker_GetResult_Response {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__TrackArucoMarker_GetResult_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TrackArucoMarker_GetResult_ResponseSeqRaw {
    data: *mut TrackArucoMarker_GetResult_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TrackArucoMarker_GetResult_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_GetResult_ResponseSeq<const N: usize> {
    data: *mut TrackArucoMarker_GetResult_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TrackArucoMarker_GetResult_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TrackArucoMarker_GetResult_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_GetResult_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TrackArucoMarker_GetResult_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TrackArucoMarker_GetResult_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TrackArucoMarker_GetResult_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TrackArucoMarker_GetResult_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TrackArucoMarker_GetResult_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TrackArucoMarker_GetResult_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = TrackArucoMarker_GetResult_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__TrackArucoMarker_GetResult_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TrackArucoMarker_GetResult_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for TrackArucoMarker_GetResult_ResponseSeq<N> {}

extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__action__TrackArucoMarker_GetResult() -> *const rcl::rosidl_service_type_support_t;
}

#[derive(Debug)]
pub struct TrackArucoMarker_GetResult;

impl ActionResult for TrackArucoMarker_GetResult {
    type Request = TrackArucoMarker_GetResult_Request;
    type Response = TrackArucoMarker_GetResult_Response;
    fn type_support() -> *const rcl::rosidl_service_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__action__TrackArucoMarker_GetResult()
        }
    }
}

impl GetUUID for TrackArucoMarker_GetResult_Request {
    fn get_uuid(&self) -> &[u8; 16] {
        &self.goal_id.uuid
    }
}

impl ResultResponse for TrackArucoMarker_GetResult_Response {
    fn get_status(&self) -> u8 {
        self.status
    }
}


#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_Feedback {
    pub marker_transform: geometry_msgs::msg::TransformStamped,
    pub marker_in_view: bool,
}

extern "C" {
    fn custom_interfaces__action__TrackArucoMarker_Feedback__init(msg: *mut TrackArucoMarker_Feedback) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Feedback__fini(msg: *mut TrackArucoMarker_Feedback);
    fn custom_interfaces__action__TrackArucoMarker_Feedback__are_equal(lhs: *const TrackArucoMarker_Feedback, rhs: *const TrackArucoMarker_Feedback) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Feedback__Sequence__init(msg: *mut TrackArucoMarker_FeedbackSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_Feedback__Sequence__fini(msg: *mut TrackArucoMarker_FeedbackSeqRaw);
    fn custom_interfaces__action__TrackArucoMarker_Feedback__Sequence__are_equal(lhs: *const TrackArucoMarker_FeedbackSeqRaw, rhs: *const TrackArucoMarker_FeedbackSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_Feedback() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TrackArucoMarker_Feedback {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_Feedback()
        }
    }
}

impl PartialEq for TrackArucoMarker_Feedback {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__TrackArucoMarker_Feedback__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TrackArucoMarker_FeedbackSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TrackArucoMarker_FeedbackSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TrackArucoMarker_FeedbackSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__TrackArucoMarker_Feedback__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TrackArucoMarker_Feedback {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_Feedback__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TrackArucoMarker_Feedback {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__TrackArucoMarker_Feedback__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TrackArucoMarker_FeedbackSeqRaw {
    data: *mut TrackArucoMarker_Feedback,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TrackArucoMarker_Feedback.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_FeedbackSeq<const N: usize> {
    data: *mut TrackArucoMarker_Feedback,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TrackArucoMarker_FeedbackSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TrackArucoMarker_FeedbackSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_Feedback__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TrackArucoMarker_FeedbackSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TrackArucoMarker_Feedback] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TrackArucoMarker_Feedback] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TrackArucoMarker_Feedback> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TrackArucoMarker_Feedback> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TrackArucoMarker_FeedbackSeq<N> {
    fn drop(&mut self) {
        let mut msg = TrackArucoMarker_FeedbackSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__TrackArucoMarker_Feedback__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TrackArucoMarker_FeedbackSeq<N> {}
unsafe impl<const N: usize> Sync for TrackArucoMarker_FeedbackSeq<N> {}

extern "C" {
    fn custom_interfaces__action__TrackArucoMarker_FeedbackMessage__init(msg: *mut TrackArucoMarker_FeedbackMessage) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_FeedbackMessage__fini(msg: *mut TrackArucoMarker_FeedbackMessage);
    fn custom_interfaces__action__TrackArucoMarker_FeedbackMessage__are_equal(lhs: *const TrackArucoMarker_FeedbackMessage, rhs: *const TrackArucoMarker_FeedbackMessage) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_FeedbackMessage__Sequence__init(msg: *mut TrackArucoMarker_FeedbackMessageSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__TrackArucoMarker_FeedbackMessage__Sequence__fini(msg: *mut TrackArucoMarker_FeedbackMessageSeqRaw);
    fn custom_interfaces__action__TrackArucoMarker_FeedbackMessage__Sequence__are_equal(lhs: *const TrackArucoMarker_FeedbackMessageSeqRaw, rhs: *const TrackArucoMarker_FeedbackMessageSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_FeedbackMessage() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for TrackArucoMarker_FeedbackMessage {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__TrackArucoMarker_FeedbackMessage()
        }
    }
}

impl PartialEq for TrackArucoMarker_FeedbackMessage {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__TrackArucoMarker_FeedbackMessage__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for TrackArucoMarker_FeedbackMessageSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = TrackArucoMarker_FeedbackMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = TrackArucoMarker_FeedbackMessageSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__TrackArucoMarker_FeedbackMessage__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl TrackArucoMarker_FeedbackMessage {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_FeedbackMessage__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for TrackArucoMarker_FeedbackMessage {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__TrackArucoMarker_FeedbackMessage__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct TrackArucoMarker_FeedbackMessageSeqRaw {
    data: *mut TrackArucoMarker_FeedbackMessage,
    size: size_t,
    capacity: size_t,
}

/// Sequence of TrackArucoMarker_FeedbackMessage.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct TrackArucoMarker_FeedbackMessageSeq<const N: usize> {
    data: *mut TrackArucoMarker_FeedbackMessage,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> TrackArucoMarker_FeedbackMessageSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: TrackArucoMarker_FeedbackMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__TrackArucoMarker_FeedbackMessage__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: TrackArucoMarker_FeedbackMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[TrackArucoMarker_FeedbackMessage] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [TrackArucoMarker_FeedbackMessage] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, TrackArucoMarker_FeedbackMessage> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, TrackArucoMarker_FeedbackMessage> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for TrackArucoMarker_FeedbackMessageSeq<N> {
    fn drop(&mut self) {
        let mut msg = TrackArucoMarker_FeedbackMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__TrackArucoMarker_FeedbackMessage__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for TrackArucoMarker_FeedbackMessageSeq<N> {}
unsafe impl<const N: usize> Sync for TrackArucoMarker_FeedbackMessageSeq<N> {}

impl GetUUID for TrackArucoMarker_FeedbackMessage {
    fn get_uuid(&self) -> &[u8; 16] {
        &self.goal_id.uuid
    }
}
