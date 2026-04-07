use safe_drive::{msg::{ActionMsg, ActionGoal, ActionResult, GetUUID, GoalResponse, ResultResponse, TypeSupport, builtin_interfaces::UnsafeTime, unique_identifier_msgs}, rcl::{self, size_t}};

extern "C" {
    fn rosidl_typesupport_c__get_action_type_support_handle__custom_interfaces__action__FindArucoWithPose() -> *const rcl::rosidl_action_type_support_t;
}

#[derive(Debug)]
pub struct FindArucoWithPose;

impl ActionMsg for FindArucoWithPose {
    type Goal = FindArucoWithPose_SendGoal;
    type Result = FindArucoWithPose_GetResult;
    type Feedback = FindArucoWithPose_FeedbackMessage;
    fn type_support() -> *const rcl::rosidl_action_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_action_type_support_handle__custom_interfaces__action__FindArucoWithPose()
        }
    }

    type GoalContent = FindArucoWithPose_Goal;

    fn new_goal_request(
        goal: Self::GoalContent,
        uuid: [u8; 16],
    ) -> <Self::Goal as ActionGoal>::Request {
        FindArucoWithPose_SendGoal_Request {
            goal,
            goal_id: unique_identifier_msgs::msg::UUID { uuid },
        }
    }

    type ResultContent = FindArucoWithPose_Result;

    fn new_result_response(
        status: u8,
        result: Self::ResultContent,
    ) -> <Self::Result as ActionResult>::Response {
        FindArucoWithPose_GetResult_Response { status, result }
    }

    type FeedbackContent = FindArucoWithPose_Feedback;

    fn new_feedback_message(feedback: Self::FeedbackContent, uuid: [u8; 16]) -> Self::Feedback {
        FindArucoWithPose_FeedbackMessage {
            feedback,
            goal_id: unique_identifier_msgs::msg::UUID { uuid },
        }
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_SendGoal_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub goal: FindArucoWithPose_Goal,
}

#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_SendGoal_Response {
    pub accepted: bool,
    pub stamp: UnsafeTime,
}

#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_GetResult_Request {
    pub goal_id: unique_identifier_msgs::msg::UUID,
}

#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_GetResult_Response {
    pub status: u8,
    pub result: FindArucoWithPose_Result,
}

#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_FeedbackMessage {
    pub goal_id: unique_identifier_msgs::msg::UUID,
    pub feedback: FindArucoWithPose_Feedback,
}

#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_Goal {
    pub structure_needs_at_least_one_member: u8,
}

extern "C" {
    fn custom_interfaces__action__FindArucoWithPose_Goal__init(msg: *mut FindArucoWithPose_Goal) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Goal__fini(msg: *mut FindArucoWithPose_Goal);
    fn custom_interfaces__action__FindArucoWithPose_Goal__are_equal(lhs: *const FindArucoWithPose_Goal, rhs: *const FindArucoWithPose_Goal) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Goal__Sequence__init(msg: *mut FindArucoWithPose_GoalSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Goal__Sequence__fini(msg: *mut FindArucoWithPose_GoalSeqRaw);
    fn custom_interfaces__action__FindArucoWithPose_Goal__Sequence__are_equal(lhs: *const FindArucoWithPose_GoalSeqRaw, rhs: *const FindArucoWithPose_GoalSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_Goal() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for FindArucoWithPose_Goal {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_Goal()
        }
    }
}

impl PartialEq for FindArucoWithPose_Goal {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__FindArucoWithPose_Goal__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for FindArucoWithPose_GoalSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = FindArucoWithPose_GoalSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = FindArucoWithPose_GoalSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__FindArucoWithPose_Goal__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl FindArucoWithPose_Goal {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_Goal__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for FindArucoWithPose_Goal {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__FindArucoWithPose_Goal__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct FindArucoWithPose_GoalSeqRaw {
    data: *mut FindArucoWithPose_Goal,
    size: size_t,
    capacity: size_t,
}

/// Sequence of FindArucoWithPose_Goal.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_GoalSeq<const N: usize> {
    data: *mut FindArucoWithPose_Goal,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> FindArucoWithPose_GoalSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: FindArucoWithPose_GoalSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_Goal__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: FindArucoWithPose_GoalSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[FindArucoWithPose_Goal] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [FindArucoWithPose_Goal] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, FindArucoWithPose_Goal> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, FindArucoWithPose_Goal> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for FindArucoWithPose_GoalSeq<N> {
    fn drop(&mut self) {
        let mut msg = FindArucoWithPose_GoalSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__FindArucoWithPose_Goal__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for FindArucoWithPose_GoalSeq<N> {}
unsafe impl<const N: usize> Sync for FindArucoWithPose_GoalSeq<N> {}

extern "C" {
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Request__init(msg: *mut FindArucoWithPose_SendGoal_Request) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Request__fini(msg: *mut FindArucoWithPose_SendGoal_Request);
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Request__are_equal(lhs: *const FindArucoWithPose_SendGoal_Request, rhs: *const FindArucoWithPose_SendGoal_Request) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Request__Sequence__init(msg: *mut FindArucoWithPose_SendGoal_RequestSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Request__Sequence__fini(msg: *mut FindArucoWithPose_SendGoal_RequestSeqRaw);
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Request__Sequence__are_equal(lhs: *const FindArucoWithPose_SendGoal_RequestSeqRaw, rhs: *const FindArucoWithPose_SendGoal_RequestSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_SendGoal_Request() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for FindArucoWithPose_SendGoal_Request {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_SendGoal_Request()
        }
    }
}

impl PartialEq for FindArucoWithPose_SendGoal_Request {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__FindArucoWithPose_SendGoal_Request__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for FindArucoWithPose_SendGoal_RequestSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = FindArucoWithPose_SendGoal_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = FindArucoWithPose_SendGoal_RequestSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__FindArucoWithPose_SendGoal_Request__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl FindArucoWithPose_SendGoal_Request {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_SendGoal_Request__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for FindArucoWithPose_SendGoal_Request {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__FindArucoWithPose_SendGoal_Request__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct FindArucoWithPose_SendGoal_RequestSeqRaw {
    data: *mut FindArucoWithPose_SendGoal_Request,
    size: size_t,
    capacity: size_t,
}

/// Sequence of FindArucoWithPose_SendGoal_Request.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_SendGoal_RequestSeq<const N: usize> {
    data: *mut FindArucoWithPose_SendGoal_Request,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> FindArucoWithPose_SendGoal_RequestSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: FindArucoWithPose_SendGoal_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_SendGoal_Request__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: FindArucoWithPose_SendGoal_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[FindArucoWithPose_SendGoal_Request] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [FindArucoWithPose_SendGoal_Request] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, FindArucoWithPose_SendGoal_Request> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, FindArucoWithPose_SendGoal_Request> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for FindArucoWithPose_SendGoal_RequestSeq<N> {
    fn drop(&mut self) {
        let mut msg = FindArucoWithPose_SendGoal_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__FindArucoWithPose_SendGoal_Request__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for FindArucoWithPose_SendGoal_RequestSeq<N> {}
unsafe impl<const N: usize> Sync for FindArucoWithPose_SendGoal_RequestSeq<N> {}

extern "C" {
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Response__init(msg: *mut FindArucoWithPose_SendGoal_Response) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Response__fini(msg: *mut FindArucoWithPose_SendGoal_Response);
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Response__are_equal(lhs: *const FindArucoWithPose_SendGoal_Response, rhs: *const FindArucoWithPose_SendGoal_Response) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Response__Sequence__init(msg: *mut FindArucoWithPose_SendGoal_ResponseSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Response__Sequence__fini(msg: *mut FindArucoWithPose_SendGoal_ResponseSeqRaw);
    fn custom_interfaces__action__FindArucoWithPose_SendGoal_Response__Sequence__are_equal(lhs: *const FindArucoWithPose_SendGoal_ResponseSeqRaw, rhs: *const FindArucoWithPose_SendGoal_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_SendGoal_Response() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for FindArucoWithPose_SendGoal_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_SendGoal_Response()
        }
    }
}

impl PartialEq for FindArucoWithPose_SendGoal_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__FindArucoWithPose_SendGoal_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for FindArucoWithPose_SendGoal_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = FindArucoWithPose_SendGoal_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = FindArucoWithPose_SendGoal_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__FindArucoWithPose_SendGoal_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl FindArucoWithPose_SendGoal_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_SendGoal_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for FindArucoWithPose_SendGoal_Response {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__FindArucoWithPose_SendGoal_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct FindArucoWithPose_SendGoal_ResponseSeqRaw {
    data: *mut FindArucoWithPose_SendGoal_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of FindArucoWithPose_SendGoal_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_SendGoal_ResponseSeq<const N: usize> {
    data: *mut FindArucoWithPose_SendGoal_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> FindArucoWithPose_SendGoal_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: FindArucoWithPose_SendGoal_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_SendGoal_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: FindArucoWithPose_SendGoal_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[FindArucoWithPose_SendGoal_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [FindArucoWithPose_SendGoal_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, FindArucoWithPose_SendGoal_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, FindArucoWithPose_SendGoal_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for FindArucoWithPose_SendGoal_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = FindArucoWithPose_SendGoal_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__FindArucoWithPose_SendGoal_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for FindArucoWithPose_SendGoal_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for FindArucoWithPose_SendGoal_ResponseSeq<N> {}

extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__action__FindArucoWithPose_SendGoal() -> *const rcl::rosidl_service_type_support_t;
}

#[derive(Debug)]
pub struct FindArucoWithPose_SendGoal;

impl ActionGoal for FindArucoWithPose_SendGoal {
    type Request = FindArucoWithPose_SendGoal_Request;
    type Response = FindArucoWithPose_SendGoal_Response;
    fn type_support() -> *const rcl::rosidl_service_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__action__FindArucoWithPose_SendGoal()
        }
    }
}

impl GetUUID for FindArucoWithPose_SendGoal_Request {
    fn get_uuid(&self) -> &[u8; 16] {
        &self.goal_id.uuid
    }
}

impl GoalResponse for FindArucoWithPose_SendGoal_Response {
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
pub struct FindArucoWithPose_Result {
    pub structure_needs_at_least_one_member: u8,
}

extern "C" {
    fn custom_interfaces__action__FindArucoWithPose_Result__init(msg: *mut FindArucoWithPose_Result) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Result__fini(msg: *mut FindArucoWithPose_Result);
    fn custom_interfaces__action__FindArucoWithPose_Result__are_equal(lhs: *const FindArucoWithPose_Result, rhs: *const FindArucoWithPose_Result) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Result__Sequence__init(msg: *mut FindArucoWithPose_ResultSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Result__Sequence__fini(msg: *mut FindArucoWithPose_ResultSeqRaw);
    fn custom_interfaces__action__FindArucoWithPose_Result__Sequence__are_equal(lhs: *const FindArucoWithPose_ResultSeqRaw, rhs: *const FindArucoWithPose_ResultSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_Result() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for FindArucoWithPose_Result {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_Result()
        }
    }
}

impl PartialEq for FindArucoWithPose_Result {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__FindArucoWithPose_Result__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for FindArucoWithPose_ResultSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = FindArucoWithPose_ResultSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = FindArucoWithPose_ResultSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__FindArucoWithPose_Result__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl FindArucoWithPose_Result {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_Result__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for FindArucoWithPose_Result {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__FindArucoWithPose_Result__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct FindArucoWithPose_ResultSeqRaw {
    data: *mut FindArucoWithPose_Result,
    size: size_t,
    capacity: size_t,
}

/// Sequence of FindArucoWithPose_Result.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_ResultSeq<const N: usize> {
    data: *mut FindArucoWithPose_Result,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> FindArucoWithPose_ResultSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: FindArucoWithPose_ResultSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_Result__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: FindArucoWithPose_ResultSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[FindArucoWithPose_Result] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [FindArucoWithPose_Result] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, FindArucoWithPose_Result> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, FindArucoWithPose_Result> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for FindArucoWithPose_ResultSeq<N> {
    fn drop(&mut self) {
        let mut msg = FindArucoWithPose_ResultSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__FindArucoWithPose_Result__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for FindArucoWithPose_ResultSeq<N> {}
unsafe impl<const N: usize> Sync for FindArucoWithPose_ResultSeq<N> {}

extern "C" {
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Request__init(msg: *mut FindArucoWithPose_GetResult_Request) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Request__fini(msg: *mut FindArucoWithPose_GetResult_Request);
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Request__are_equal(lhs: *const FindArucoWithPose_GetResult_Request, rhs: *const FindArucoWithPose_GetResult_Request) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Request__Sequence__init(msg: *mut FindArucoWithPose_GetResult_RequestSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Request__Sequence__fini(msg: *mut FindArucoWithPose_GetResult_RequestSeqRaw);
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Request__Sequence__are_equal(lhs: *const FindArucoWithPose_GetResult_RequestSeqRaw, rhs: *const FindArucoWithPose_GetResult_RequestSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_GetResult_Request() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for FindArucoWithPose_GetResult_Request {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_GetResult_Request()
        }
    }
}

impl PartialEq for FindArucoWithPose_GetResult_Request {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__FindArucoWithPose_GetResult_Request__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for FindArucoWithPose_GetResult_RequestSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = FindArucoWithPose_GetResult_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = FindArucoWithPose_GetResult_RequestSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__FindArucoWithPose_GetResult_Request__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl FindArucoWithPose_GetResult_Request {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_GetResult_Request__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for FindArucoWithPose_GetResult_Request {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__FindArucoWithPose_GetResult_Request__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct FindArucoWithPose_GetResult_RequestSeqRaw {
    data: *mut FindArucoWithPose_GetResult_Request,
    size: size_t,
    capacity: size_t,
}

/// Sequence of FindArucoWithPose_GetResult_Request.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_GetResult_RequestSeq<const N: usize> {
    data: *mut FindArucoWithPose_GetResult_Request,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> FindArucoWithPose_GetResult_RequestSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: FindArucoWithPose_GetResult_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_GetResult_Request__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: FindArucoWithPose_GetResult_RequestSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[FindArucoWithPose_GetResult_Request] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [FindArucoWithPose_GetResult_Request] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, FindArucoWithPose_GetResult_Request> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, FindArucoWithPose_GetResult_Request> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for FindArucoWithPose_GetResult_RequestSeq<N> {
    fn drop(&mut self) {
        let mut msg = FindArucoWithPose_GetResult_RequestSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__FindArucoWithPose_GetResult_Request__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for FindArucoWithPose_GetResult_RequestSeq<N> {}
unsafe impl<const N: usize> Sync for FindArucoWithPose_GetResult_RequestSeq<N> {}

extern "C" {
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Response__init(msg: *mut FindArucoWithPose_GetResult_Response) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Response__fini(msg: *mut FindArucoWithPose_GetResult_Response);
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Response__are_equal(lhs: *const FindArucoWithPose_GetResult_Response, rhs: *const FindArucoWithPose_GetResult_Response) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Response__Sequence__init(msg: *mut FindArucoWithPose_GetResult_ResponseSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Response__Sequence__fini(msg: *mut FindArucoWithPose_GetResult_ResponseSeqRaw);
    fn custom_interfaces__action__FindArucoWithPose_GetResult_Response__Sequence__are_equal(lhs: *const FindArucoWithPose_GetResult_ResponseSeqRaw, rhs: *const FindArucoWithPose_GetResult_ResponseSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_GetResult_Response() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for FindArucoWithPose_GetResult_Response {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_GetResult_Response()
        }
    }
}

impl PartialEq for FindArucoWithPose_GetResult_Response {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__FindArucoWithPose_GetResult_Response__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for FindArucoWithPose_GetResult_ResponseSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = FindArucoWithPose_GetResult_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = FindArucoWithPose_GetResult_ResponseSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__FindArucoWithPose_GetResult_Response__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl FindArucoWithPose_GetResult_Response {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_GetResult_Response__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for FindArucoWithPose_GetResult_Response {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__FindArucoWithPose_GetResult_Response__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct FindArucoWithPose_GetResult_ResponseSeqRaw {
    data: *mut FindArucoWithPose_GetResult_Response,
    size: size_t,
    capacity: size_t,
}

/// Sequence of FindArucoWithPose_GetResult_Response.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_GetResult_ResponseSeq<const N: usize> {
    data: *mut FindArucoWithPose_GetResult_Response,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> FindArucoWithPose_GetResult_ResponseSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: FindArucoWithPose_GetResult_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_GetResult_Response__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: FindArucoWithPose_GetResult_ResponseSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[FindArucoWithPose_GetResult_Response] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [FindArucoWithPose_GetResult_Response] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, FindArucoWithPose_GetResult_Response> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, FindArucoWithPose_GetResult_Response> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for FindArucoWithPose_GetResult_ResponseSeq<N> {
    fn drop(&mut self) {
        let mut msg = FindArucoWithPose_GetResult_ResponseSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__FindArucoWithPose_GetResult_Response__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for FindArucoWithPose_GetResult_ResponseSeq<N> {}
unsafe impl<const N: usize> Sync for FindArucoWithPose_GetResult_ResponseSeq<N> {}

extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__action__FindArucoWithPose_GetResult() -> *const rcl::rosidl_service_type_support_t;
}

#[derive(Debug)]
pub struct FindArucoWithPose_GetResult;

impl ActionResult for FindArucoWithPose_GetResult {
    type Request = FindArucoWithPose_GetResult_Request;
    type Response = FindArucoWithPose_GetResult_Response;
    fn type_support() -> *const rcl::rosidl_service_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_service_type_support_handle__custom_interfaces__action__FindArucoWithPose_GetResult()
        }
    }
}

impl GetUUID for FindArucoWithPose_GetResult_Request {
    fn get_uuid(&self) -> &[u8; 16] {
        &self.goal_id.uuid
    }
}

impl ResultResponse for FindArucoWithPose_GetResult_Response {
    fn get_status(&self) -> u8 {
        self.status
    }
}


#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_Feedback {
    pub marker_poses: geometry_msgs::msg::PoseSeq<0>,
    pub marker_ids: safe_drive::msg::U8Seq<0>,
    pub time_last_image_arrived: builtin_interfaces::msg::Time,
}

extern "C" {
    fn custom_interfaces__action__FindArucoWithPose_Feedback__init(msg: *mut FindArucoWithPose_Feedback) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Feedback__fini(msg: *mut FindArucoWithPose_Feedback);
    fn custom_interfaces__action__FindArucoWithPose_Feedback__are_equal(lhs: *const FindArucoWithPose_Feedback, rhs: *const FindArucoWithPose_Feedback) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Feedback__Sequence__init(msg: *mut FindArucoWithPose_FeedbackSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_Feedback__Sequence__fini(msg: *mut FindArucoWithPose_FeedbackSeqRaw);
    fn custom_interfaces__action__FindArucoWithPose_Feedback__Sequence__are_equal(lhs: *const FindArucoWithPose_FeedbackSeqRaw, rhs: *const FindArucoWithPose_FeedbackSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_Feedback() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for FindArucoWithPose_Feedback {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_Feedback()
        }
    }
}

impl PartialEq for FindArucoWithPose_Feedback {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__FindArucoWithPose_Feedback__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for FindArucoWithPose_FeedbackSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = FindArucoWithPose_FeedbackSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = FindArucoWithPose_FeedbackSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__FindArucoWithPose_Feedback__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl FindArucoWithPose_Feedback {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_Feedback__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for FindArucoWithPose_Feedback {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__FindArucoWithPose_Feedback__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct FindArucoWithPose_FeedbackSeqRaw {
    data: *mut FindArucoWithPose_Feedback,
    size: size_t,
    capacity: size_t,
}

/// Sequence of FindArucoWithPose_Feedback.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_FeedbackSeq<const N: usize> {
    data: *mut FindArucoWithPose_Feedback,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> FindArucoWithPose_FeedbackSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: FindArucoWithPose_FeedbackSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_Feedback__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: FindArucoWithPose_FeedbackSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[FindArucoWithPose_Feedback] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [FindArucoWithPose_Feedback] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, FindArucoWithPose_Feedback> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, FindArucoWithPose_Feedback> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for FindArucoWithPose_FeedbackSeq<N> {
    fn drop(&mut self) {
        let mut msg = FindArucoWithPose_FeedbackSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__FindArucoWithPose_Feedback__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for FindArucoWithPose_FeedbackSeq<N> {}
unsafe impl<const N: usize> Sync for FindArucoWithPose_FeedbackSeq<N> {}

extern "C" {
    fn custom_interfaces__action__FindArucoWithPose_FeedbackMessage__init(msg: *mut FindArucoWithPose_FeedbackMessage) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_FeedbackMessage__fini(msg: *mut FindArucoWithPose_FeedbackMessage);
    fn custom_interfaces__action__FindArucoWithPose_FeedbackMessage__are_equal(lhs: *const FindArucoWithPose_FeedbackMessage, rhs: *const FindArucoWithPose_FeedbackMessage) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_FeedbackMessage__Sequence__init(msg: *mut FindArucoWithPose_FeedbackMessageSeqRaw, size: usize) -> bool;
    fn custom_interfaces__action__FindArucoWithPose_FeedbackMessage__Sequence__fini(msg: *mut FindArucoWithPose_FeedbackMessageSeqRaw);
    fn custom_interfaces__action__FindArucoWithPose_FeedbackMessage__Sequence__are_equal(lhs: *const FindArucoWithPose_FeedbackMessageSeqRaw, rhs: *const FindArucoWithPose_FeedbackMessageSeqRaw) -> bool;
    fn rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_FeedbackMessage() -> *const rcl::rosidl_message_type_support_t;
}

impl TypeSupport for FindArucoWithPose_FeedbackMessage {
    fn type_support() -> *const rcl::rosidl_message_type_support_t {
        unsafe {
            rosidl_typesupport_c__get_message_type_support_handle__custom_interfaces__action__FindArucoWithPose_FeedbackMessage()
        }
    }
}

impl PartialEq for FindArucoWithPose_FeedbackMessage {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            custom_interfaces__action__FindArucoWithPose_FeedbackMessage__are_equal(self, other)
        }
    }
}

impl<const N: usize> PartialEq for FindArucoWithPose_FeedbackMessageSeq<N> {
    fn eq(&self, other: &Self) -> bool {
        unsafe {
            let msg1 = FindArucoWithPose_FeedbackMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
            let msg2 = FindArucoWithPose_FeedbackMessageSeqRaw{ data: other.data, size: other.size, capacity: other.capacity };
            custom_interfaces__action__FindArucoWithPose_FeedbackMessage__Sequence__are_equal(&msg1, &msg2)
        }
    }
}

impl FindArucoWithPose_FeedbackMessage {
    pub fn new() -> Option<Self> {
        let mut msg: Self = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_FeedbackMessage__init(&mut msg) } {
            Some(msg)
        } else {
            None
        }
    }
}

impl Drop for FindArucoWithPose_FeedbackMessage {
    fn drop(&mut self) {
        unsafe { custom_interfaces__action__FindArucoWithPose_FeedbackMessage__fini(self) };
    }
}

#[repr(C)]
#[derive(Debug)]
struct FindArucoWithPose_FeedbackMessageSeqRaw {
    data: *mut FindArucoWithPose_FeedbackMessage,
    size: size_t,
    capacity: size_t,
}

/// Sequence of FindArucoWithPose_FeedbackMessage.
/// `N` is the maximum number of elements.
/// If `N` is `0`, the size is unlimited.
#[repr(C)]
#[derive(Debug)]
pub struct FindArucoWithPose_FeedbackMessageSeq<const N: usize> {
    data: *mut FindArucoWithPose_FeedbackMessage,
    size: size_t,
    capacity: size_t,
}

impl<const N: usize> FindArucoWithPose_FeedbackMessageSeq<N> {
    /// Create a sequence of.
    /// `N` represents the maximum number of elements.
    /// If `N` is `0`, the sequence is unlimited.
    pub fn new(size: usize) -> Option<Self> {
        if N != 0 && size > N {
            // the size exceeds in the maximum number
            return None;
        }
        let mut msg: FindArucoWithPose_FeedbackMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        if unsafe { custom_interfaces__action__FindArucoWithPose_FeedbackMessage__Sequence__init(&mut msg, size) } {
            Some(Self { data: msg.data, size: msg.size, capacity: msg.capacity })
        } else {
            None
        }
    }

    pub fn null() -> Self {
        let msg: FindArucoWithPose_FeedbackMessageSeqRaw = unsafe { std::mem::MaybeUninit::zeroed().assume_init() };
        Self { data: msg.data, size: msg.size, capacity: msg.capacity }
    }

    pub fn as_slice(&self) -> &[FindArucoWithPose_FeedbackMessage] {
        if self.data.is_null() {
            &[]
        } else {
            let s = unsafe { std::slice::from_raw_parts(self.data, self.size as _) };
            s
        }
    }

    pub fn as_slice_mut(&mut self) -> &mut [FindArucoWithPose_FeedbackMessage] {
        if self.data.is_null() {
            &mut []
        } else {
            let s = unsafe { std::slice::from_raw_parts_mut(self.data, self.size as _) };
            s
        }
    }

    pub fn iter(&self) -> std::slice::Iter<'_, FindArucoWithPose_FeedbackMessage> {
        self.as_slice().iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, FindArucoWithPose_FeedbackMessage> {
        self.as_slice_mut().iter_mut()
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl<const N: usize> Drop for FindArucoWithPose_FeedbackMessageSeq<N> {
    fn drop(&mut self) {
        let mut msg = FindArucoWithPose_FeedbackMessageSeqRaw{ data: self.data, size: self.size, capacity: self.capacity };
        unsafe { custom_interfaces__action__FindArucoWithPose_FeedbackMessage__Sequence__fini(&mut msg) };
    }
}

unsafe impl<const N: usize> Send for FindArucoWithPose_FeedbackMessageSeq<N> {}
unsafe impl<const N: usize> Sync for FindArucoWithPose_FeedbackMessageSeq<N> {}

impl GetUUID for FindArucoWithPose_FeedbackMessage {
    fn get_uuid(&self) -> &[u8; 16] {
        &self.goal_id.uuid
    }
}
