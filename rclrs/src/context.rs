use crate::error::{RclReturnCode, ToResult};
use crate::rcl_bindings::*;
use crate::Node;
use alloc::sync::Arc;
use alloc::vec::Vec;
use cstr_core::{c_char, CString};

#[cfg(not(feature = "std"))]
use spin::{Mutex, MutexGuard};

#[cfg(feature = "std")]
use parking_lot::{Mutex, MutexGuard};

/// The class that manages the `Context`'s C resource.
pub(crate) struct ContextHandle(Mutex<rcl_context_t>);

impl ContextHandle {
    /// Returns a mutable reference to the `rcl_context`.
    pub fn get_mut(&mut self) -> &mut rcl_context_t {
        self.0.get_mut()
    }

    /// Returns a mutex for the context's handle.
    /// 
    /// Blocks the current thread until the mutex can be acquired.
    pub fn lock(&self) -> MutexGuard<rcl_context_t> {
        self.0.lock()
    }
}

impl Drop for ContextHandle {
    fn drop(&mut self) {
        unsafe {
            rcl_shutdown(&mut *self.get_mut() as *mut _);
        }
    }
}

/// The main class for managing a ROS context.
/// 
/// "Context which encapsulates shared state between nodes and other similar entities.
/// A context also represents the lifecycle between init and shutdown of rclcpp. It is often used 
/// in conjunction with [`rclcpp::init`](https://docs.ros2.org/dashing/api/rclcpp/namespacerclcpp.html#a2db29afebba8f677bc8660a45bb910bb), 
/// or `rclcpp::init_local`, and [`rclcpp::shutdown`](https://docs.ros2.org/dashing/api/rclcpp/namespacerclcpp.html#a493714a679d1591142800416a286689f)."
/// 
/// [Source](https://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Context.html#details)
pub struct Context {
    pub(crate) handle: Arc<ContextHandle>,
}

impl Context {
    /// Initializes the context.
    /// 
    /// Returns `Ok(())` on success, otherwise returns an error.
    /// 
    /// # Errors
    /// Returns [`InvalidArgument`](error::RclReturnCode::InvalidArgument) if an 
    /// argument is invalid.
    /// 
    /// Returns [`RclError(RclErrorCode::AlreadyInit)`](error::RclErrorCode::AlreadyInit) if 
    /// the context is already initialized.
    /// 
    /// Returns [`BadAlloc`](error::RclReturnCode::BadAlloc) if the function failed 
    /// to allocate memory for the context.
    /// 
    /// Returns [`RclError(RclErrorCode::Error)`](error::RclErrorCode::Error) if there is an
    /// unspecified error.
    fn init(&self, context_env_args: Vec<CString>) -> Result<(), RclReturnCode> {
        let c_args: Vec<*const c_char> = context_env_args.iter().map(|arg| arg.as_ptr()).collect();
        let handle = &mut *self.handle.lock();

        unsafe {
            let allocator = rcutils_get_default_allocator();
            let mut init_options = rcl_get_zero_initialized_init_options();
            rcl_init_options_init(&mut init_options as *mut _, allocator);
            rcl_init(
                c_args.len() as i32,
                c_args.as_ptr(),
                &init_options as *const _,
                handle as *mut _,
            )
            .ok()?;
            rcl_init_options_fini(&mut init_options as *mut _).ok()?;
        }

        Ok(())
    }

    /// "Return a zero initialization context object."
    /// 
    /// [Source](https://docs.ros2.org/dashing/api/rcl/context_8h.html#a5ac8c6afb74f040738f03fdfdbe9bd0e)
    /// 
    /// # Panics
    /// 
    /// Panics if [`Context::init`] returns an error.
    pub fn default(args: Vec<CString>) -> Self {
        let context = Self {
            handle: Arc::new(ContextHandle(Mutex::new(unsafe {
                rcl_get_zero_initialized_context()
            }))),
        };
        context.init(args).unwrap(); // If we can't initialize the context, ROS 2 cannot function
        context
    }

    /// "Return `true` if the given context is currently valid, otherwise `false`.
    /// 
    /// If context is `NULL`, then `false` is returned.
    /// If context is zero-initialized, then `false` is returned. 
    /// If context is uninitialized, then it is undefined behavior."
    /// 
    /// [Source](https://docs.ros2.org/dashing/api/rcl/context_8h.html#a70fd536d6f0356e2c1a183b93f67fa4c)
    pub fn ok(&self) -> Result<bool, RclReturnCode> {
        let handle = &mut *self.handle.lock();
        unsafe { Ok(rcl_context_is_valid(handle as *mut _)) }
    }

    // TODO: documentation
    pub fn create_node(&self, node_name: &str) -> Result<Node, RclReturnCode> {
        Node::new(node_name, self)
    }
}
