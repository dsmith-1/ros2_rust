use crate::error::{RclReturnCode, ToResult};
use crate::qos::QoSProfile;
use crate::rcl_bindings::*;
use crate::{Node, NodeHandle};
use alloc::sync::Arc;
use core::borrow::Borrow;
use core::marker::PhantomData;
use cstr_core::CString;
use rclrs_msg_utilities::traits::MessageDefinition;

#[cfg(not(feature = "std"))]
use spin::{Mutex, MutexGuard};

#[cfg(feature = "std")]
use parking_lot::{Mutex, MutexGuard};

/// The class that manages the `Publisher`'s C resource.
pub struct PublisherHandle {
    /// The `PublisherHandle`'s C resource manager.
    handle: Mutex<rcl_publisher_t>,

    /// A thread-safe reference to the node that the `PublisherHandle` is attached to.
    node_handle: Arc<NodeHandle>,
}

impl PublisherHandle {
    /// Returns a reference to the `PublisherHandle`'s `NodeHandle`.
    fn node_handle(&self) -> &NodeHandle {
        self.node_handle.borrow()
    }

    /// Returns a mutable reference to `self.handle`.
    fn get_mut(&mut self) -> &mut rcl_publisher_t {
        self.handle.get_mut()
    }

    /// Returns a mutex for `self.handle`.
    /// 
    /// Blocks the current thread until the mutex can be acquired.
    fn lock(&self) -> MutexGuard<rcl_publisher_t> {
        self.handle.lock()
    }

    /// Returns a mutex for `self.handle` if it can be acquired. Otherwise, `None` is
    /// returned.
    /// 
    /// Non-blocking.
    fn try_lock(&self) -> Option<MutexGuard<rcl_publisher_t>> {
        self.handle.try_lock()
    }
}

impl Drop for PublisherHandle {
    fn drop(&mut self) {
        let handle = self.handle.get_mut();
        let node_handle = &mut *self.node_handle.lock();
        unsafe {
            rcl_publisher_fini(handle as *mut _, node_handle as *mut _);
        }
    }
}

/// Main class responsible for publishing data to ROS topics.
pub struct Publisher<T>
where
    T: MessageDefinition<T>,
{
    /// A thread-safe reference to the `Publisher`'s C resource manager.
    pub handle: Arc<PublisherHandle>,
    
    /// A `PhantomData<T>` instance, where `T` is the message type that the `Publisher`
    /// can send.
    message: PhantomData<T>,
}

impl<T> Publisher<T>
where
    T: MessageDefinition<T>,
{
    /// Creates a new publisher.
    /// 
    /// Returns `Ok(Publisher<T>)` on success, otherwise returns an error.
    /// 
    /// # Errors
    /// 
    /// Returns [`InvalidArgument`](error::RclReturnCode::InvalidArgument) if an 
    /// argument is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::AlreadyInit)`](error::RclErrorCode::AlreadyInit) if 
    /// the publisher is already initialized.
    /// 
    /// Returns [`NodeError(NodeErrorCode::NodeInvalid)`](error::NodeErrorCode::NodeInvalid) 
    /// if the `node` is invalid.
    /// 
    /// Returns [`BadAlloc`](error::RclReturnCode::BadAlloc) if the function failed 
    /// to allocate memory for the publisher.
    /// 
    /// Returns [`RclError(RclErrorCode::TopicNameInvalid)`](error::RclErrorCode::TopicNameInvalid) if 
    /// the topic name is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::Error)`](error::RclErrorCode::Error) if there is an
    /// unspecified error.
    pub fn new(node: &Node, topic: &str, qos: QoSProfile) -> Result<Self, RclReturnCode>
    where
        T: MessageDefinition<T>,
    {
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let type_support = T::get_type_support() as *const rosidl_message_type_support_t;
        let topic_c_string = CString::new(topic).unwrap();
        let node_handle = &mut *node.handle.lock();

        unsafe {
            let mut publisher_options = rcl_publisher_get_default_options();
            publisher_options.qos = qos.into();

            rcl_publisher_init(
                &mut publisher_handle as *mut _,
                node_handle as *mut _,
                type_support,
                topic_c_string.as_ptr(),
                &publisher_options as *const _,
            )
            .ok()?;
        }
 
        let handle = Arc::new(PublisherHandle {
            handle: Mutex::new(publisher_handle),
            node_handle: node.handle.clone(),
        });

        Ok(Self {
            handle,
            message: PhantomData,
        })
    }

    /// Publishes a message.
    /// 
    /// Returns `Ok(())` on success, otherwise returns an error.
    /// 
    /// # Errors
    /// 
    /// Returns [`RclError(RclErrorCode::PublisherInvalid)`](error::RclErrorCode::PublisherInvalid)
    /// if the publisher handle is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::Error)`](error::RclErrorCode::Error) if there is an
    /// unspecified error.
    pub fn publish(&self, message: &T) -> Result<(), RclReturnCode> {
        let native_message_ptr = message.get_native_message();
        let handle = &mut *self.handle.lock();
        let ret = unsafe {
            rcl_publish(
                handle as *mut _,
                native_message_ptr as *mut _,
                core::ptr::null_mut(),
            )
        };
        message.destroy_native_message(native_message_ptr);
        ret.ok()
    }
}
