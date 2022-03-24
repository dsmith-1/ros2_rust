use alloc::{
    sync::{Arc, Weak},
    vec::Vec,
};

use crate::error::{RclReturnCode, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::{Context, ContextHandle};

use rosidl_runtime_rs::Message;

use cstr_core::CString;

pub mod publisher;
pub use self::publisher::*;
pub mod subscription;
pub use self::subscription::*;

#[cfg(not(feature = "std"))]
use spin::{Mutex, MutexGuard};

#[cfg(feature = "std")]
use parking_lot::{Mutex, MutexGuard};

/// The class that manages the `Node`'s C resource.
pub struct NodeHandle(Mutex<rcl_node_t>);

impl NodeHandle {

    /// Returns a mutex for `rcl_node`.
    /// 
    /// Blocks the current thread until the mutex can be acquired.
    pub fn lock(&self) -> MutexGuard<rcl_node_t> {
        self.0.lock()
    }
}

impl Drop for NodeHandle {
    fn drop(&mut self) {
        let handle = &mut *self.0.get_mut();
        unsafe { rcl_node_fini(handle as *mut _).unwrap() };
    }
}

/// Main class for managing a ROS node.
pub struct Node {
    /// A thread-safe reference to the `Node`'s C resource manager.
    handle: Arc<NodeHandle>,

    /// A thread-safe reference to the `Node`'s `context`.
    pub(crate) context: Arc<ContextHandle>,

    /// A vector of unowned subscriptions.
    pub(crate) subscriptions: Vec<Weak<dyn SubscriptionBase>>,
}

impl Node {
    /// T
    #[allow(clippy::new_ret_no_self)]
    pub fn new(node_name: &str, context: &Context) -> Result<Node, RclReturnCode> {
        Self::new_with_namespace(node_name, "", context)
    }

        /// Initializes a new ROS node.
    /// 
    /// Returns `Ok(Node)` on success, otherwise returns an error.
    /// 
    /// "After calling, the ROS node object can be used to create other middleware primitives like publishers, services, parameters, etc.
    /// The name of the node must not be `NULL` and adhere to naming restrictions, 
    /// see the [`rmw_validate_node_name()`](http://docs.ros2.org/latest/api/rmw/validate__node__name_8h.html#a5690a285aed9735f89ef11950b6e39e3) 
    /// function for rules.
    /// 
    /// The name of the node cannot coincide with another node of the same name. If a 
    /// node of the same name is already in the domain, it will be shutdown.
    /// 
    /// The namespace of the node should not be NULL and should also pass the 
    /// [`rmw_validate_namespace()`](http://docs.ros2.org/latest/api/rmw/validate__namespace_8h.html#a043f17d240cf13df01321b19a469ee49) function's rules.
    /// 
    /// Additionally this function allows namespaces which lack a leading forward slash. Because there is no notion 
    /// of a relative namespace, there is no difference between a namespace which lacks a forward and the same 
    /// namespace with a leasing forward slash. Therefore, a namespace like "foo/bar" is automatically changed to 
    /// "/foo/bar" by this function. Similarly, the namespace "" will implicitly become "/" which is a valid namespace.
    /// 
    /// A node contains infrastructure for ROS parameters, which include advertising publishers and service servers. 
    /// This function will create those external parameter interfaces even if parameters are not used later.
    /// 
    /// The [`NodeHandle`] given must be allocated and zero initialized. Passing [a `NodeHandle`] which has already had this function 
    /// called on it, more recently than rcl_node_fini, will fail. An allocated [`NodeHandle`] with uninitialized memory is undefined behavior."
    /// 
    /// [Source](https://docs.ros2.org/bouncy/api/rcl/node_8h.html#abbf1d973c1bffeced9659892e14926a2)
    /// 
    /// # Errors
    /// 
    /// Returns [`InvalidArgument`](error::RclReturnCode::InvalidArgument) if an 
    /// argument is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::AlreadyInit)`](error::RclErrorCode::AlreadyInit) if 
    /// the node is already initialized.
    /// 
    /// Returns [`BadAlloc`](error::RclReturnCode::BadAlloc) if the function failed 
    /// to allocate memory for the node.
    /// 
    /// Returns [`NodeError(NodeErrorCode::NodeInvalidName)`](error::NodeErrorCode::NodeInvalidName) if 
    /// the node name is invalid.
    /// 
    /// Returns [`NodeError(NodeErrorCode::NodeInvalidNamespace)`](error::NodeErrorCode::NodeInvalidNamespace) if 
    /// the namespace is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::Error)`](error::RclErrorCode::Error) if there is an
    /// unspecified error.
    pub fn new_with_namespace(
        node_name: &str,
        node_ns: &str,
        context: &Context,
    ) -> Result<Node, RclReturnCode> {
        let raw_node_name = CString::new(node_name).unwrap();
        let raw_node_ns = CString::new(node_ns).unwrap();

        let mut node_handle = unsafe { rcl_get_zero_initialized_node() };
        let context_handle = &mut *context.handle.lock();

        unsafe {
            let node_options = rcl_node_get_default_options();
            rcl_node_init(
                &mut node_handle as *mut _,
                raw_node_name.as_ptr(),
                raw_node_ns.as_ptr(),
                context_handle as *mut _,
                &node_options as *const _,
            )
            .ok()?;
        }

        let handle = Arc::new(NodeHandle(Mutex::new(node_handle)));

        Ok(Node {
            handle,
            context: context.handle.clone(),
            subscriptions: alloc::vec![],
        })
    }

    // TODO: make publisher's lifetime depend on node's lifetime
    pub fn create_publisher<T>(
        &self,
        topic: &str,
        qos: QoSProfile,
    ) -> Result<Publisher<T>, RclReturnCode>
    where
        T: Message,
    {
        Publisher::<T>::new(self, topic, qos)
    }

    // TODO: make subscription's lifetime depend on node's lifetime
    pub fn create_subscription<T, F>(
        &mut self,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Arc<Subscription<T>>, RclReturnCode>
    where
        T: Message + 'static,
        F: FnMut(&T) + Sized + 'static,
    {
        let subscription = Arc::new(Subscription::<T>::new(self, topic, qos, callback)?);
        self.subscriptions
            .push(Arc::downgrade(&subscription) as Weak<dyn SubscriptionBase>);
        Ok(subscription)
    }
}
