use crate::error::ToResult;
use crate::qos::QoSProfile;
use crate::{rcl_bindings::*, RclReturnCode};
use crate::{Node, NodeHandle};
use alloc::boxed::Box;
use alloc::sync::Arc;
use core::borrow::Borrow;
use core::marker::PhantomData;
use cstr_core::CString;
use rosidl_runtime_rs::{Message, RmwMessage};

#[cfg(not(feature = "std"))]
use spin::{Mutex, MutexGuard};

#[cfg(feature = "std")]
use parking_lot::{Mutex, MutexGuard};

/// The class that manages the `Subscription`'s C resource.
pub struct SubscriptionHandle {
    /// The `SubscriptionHandle`'s C resource manager.
    handle: Mutex<rcl_subscription_t>,

    /// A thread-safe reference to the node that the `SubscriptionHandle` is attached to.
    node_handle: Arc<NodeHandle>,
}

impl SubscriptionHandle {
    /// Returns a mutex for `self.handle`.
    /// 
    /// Blocks the current thread until the mutex can be acquired.
    pub fn lock(&self) -> MutexGuard<rcl_subscription_t> {
        self.handle.lock()
    }
}

impl Drop for SubscriptionHandle {
    fn drop(&mut self) {
        let handle = self.handle.get_mut();
        let node_handle = &mut *self.node_handle.lock();
        unsafe {
            rcl_subscription_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

/// Trait to be implemented by concrete Subscriber structs
/// See [`Subscription<T>`] for an example
pub trait SubscriptionBase {
    /// Returns a reference to he `SubscriptionHandle`'s C resource manager.
    fn handle(&self) -> &SubscriptionHandle;
    fn execute(&self) -> Result<(), RclReturnCode>;
}

/// Main class responsible for subscribing to topics and receiving data over IPC in ROS
pub struct Subscription<T>
where
    T: Message,
{
    /// A thread-safe reference to the `Subscription`'s C resource manager.
    pub handle: Arc<SubscriptionHandle>,

    /// A reference to the callback function that's called on every message the `Subscription` receives.
    /// 
    /// # Lifetimes
    /// 
    /// The callback's lifetime should last as long as we need it to
    pub callback: Mutex<Box<dyn FnMut(&T) + 'static>>,

    /// A `PhantomData<T>` instance, where `T` is the message type that the `Subscription`
    /// can receive.
    message: PhantomData<T>,
}

impl<T> Subscription<T>
where
    T: Message,
{
    /// Create a new subscription.
    /// 
    /// Returns `Ok(Subscription<T>)` on success, otherwwise returns an error.
    /// 
    /// # Errors
    /// 
    /// Returns [`InvalidArgument`](error::RclReturnCode::InvalidArgument) if an 
    /// argument is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::AlreadyInit)`](error::RclErrorCode::AlreadyInit) if 
    /// the subscription is already initialized.
    /// 
    /// Returns [`NodeError(NodeErrorCode::NodeInvalid)`](error::NodeErrorCode::NodeInvalid) 
    /// if the `node` is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::TopicNameInvalid)`](error::RclErrorCode::TopicNameInvalid) if 
    /// the topic name is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::Error)`](error::RclErrorCode::Error) if there is an
    /// unspecified error.
    pub fn new<F>(
        node: &Node,
        topic: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<Self, RclReturnCode>
    where
        T: Message,
        F: FnMut(&T) + Sized + 'static,
    {
        let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
        let type_support =
            <T as Message>::RmwMsg::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.lock();

        unsafe {
            let mut subscription_options = rcl_subscription_get_default_options();
            subscription_options.qos = qos.into();
            rcl_subscription_init(
                &mut subscription_handle as *mut _,
                node_handle as *mut _,
                type_support,
                topic_c_string.as_ptr(),
                &subscription_options as *const _,
            )
            .ok()?;
        }

        let handle = Arc::new(SubscriptionHandle {
            handle: Mutex::new(subscription_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            callback: Mutex::new(Box::new(callback)),
            message: PhantomData,
        })
    }

    /// Ask RMW for the data
    ///
    /// +-------------+
    /// | rclrs::take |
    /// +------+------+
    ///        |
    ///        |
    /// +------v------+
    /// |  rcl_take   |
    /// +------+------+
    ///        |
    ///        |
    /// +------v------+
    /// |  rmw_take   |
    /// +-------------+
    /// 
    /// 
    /// # Errors
    /// 
    /// Returns [`SubscriberError(SubscriberErrorCode::SubscriptionInvalid)`](error::SubscriberErrorCode::SubscriptionInvalid) 
    /// if the `SubscriptionHandler` is invalid.
    /// 
    /// Returns [`SubscriberError(SubscriberErrorCode::SubscriptionTakeFailed)`](error::SubscriberErrorCode::SubscriptionTakeFailed) 
    /// if there is a failure when attempting to take a message from the subscription.
    /// 
    /// Returns [`InvalidArgument`](error::RclReturnCode::InvalidArgument) if an 
    /// argument is invalid.
    pub fn take(&self) -> Result<T, RclReturnCode> {
        let mut rmw_message = <T as Message>::RmwMsg::default();
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            rcl_take(
                handle as *const _,
                &mut rmw_message as *mut <T as Message>::RmwMsg as *mut _,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
            )
        };
        ret.ok()?;
        Ok(T::from_rmw_message(rmw_message))
    }
}

impl<T> SubscriptionBase for Subscription<T>
where
    T: Message,
{
    fn handle(&self) -> &SubscriptionHandle {
        self.handle.borrow()
    }

    fn execute(&self) -> Result<(), RclReturnCode> {
        let msg = self.take()?;
        (&mut *self.callback.lock())(&msg);
        Ok(())
    }
}
