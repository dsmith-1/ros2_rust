use crate::rcl_bindings::*;

/// Descriptions taken from https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html


pub enum QoSReliabilityPolicy {
    SystemDefault = 0,
    /// Guarantee that samples are delivered, may retry multiple times.
    Reliable = 1,
    /// Attempt to deliver samples, but may lose them if the network is not robust.
    BestEffort = 2,
}

pub enum QoSHistoryPolicy {
    SystemDefault = 0,
    /// Only store up to N samples, configurable via the queue depth option.
    KeepLast = 1,
    /// Store all samples, subject to the configured resource limits of the underlying middleware.
    KeepAll = 2,
}

pub enum QoSDurabilityPolicy {
    SystemDefault = 0,
    /// The publisher becomes responsible for persisting samples for “late-joining” subscriptions.
    TransientLocal = 1,
    /// The publisher becomes responsible for persisting samples for “late-joining” subscriptions.
    Volatile = 2,
}

pub struct QoSProfile {
    pub history: QoSHistoryPolicy,
    pub depth: isize,
    pub reliability: QoSReliabilityPolicy,
    pub durability: QoSDurabilityPolicy,
    pub avoid_ros_namespace_conventions: bool,
}

/// For sensor data, in most cases it’s more important to receive readings in a timely fashion, 
/// rather than ensuring that all of them arrive. That is, developers want the latest samples 
/// as soon as they are captured, at the expense of maybe losing some. For that reason the 
/// sensor data profile uses best effort reliability and a smaller queue size.
pub const QOS_PROFILE_SENSOR_DATA: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast,
    depth: 5,
    reliability: QoSReliabilityPolicy::BestEffort,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

/// Parameters in ROS 2 are based on services, and as such have a similar profile. The 
/// difference is that parameters use a much larger queue depth so that requests do not get 
/// lost when, for example, the parameter client is unable to reach the parameter service server.
pub const QOS_PROFILE_PARAMETERS: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast,
    depth: 1000,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};


/// In order to make the transition from ROS 1 to ROS 2 easier, exercising a similar network 
/// behavior is desirable. By default, publishers and subscriptions in ROS 2 have “keep last” 
/// for history with a queue size of 10, “reliable” for reliability, “volatile” for durability, 
/// and “system default” for liveliness. Deadline, lifespan, and lease durations are also all 
/// set to “default”.
pub const QOS_PROFILE_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast,
    depth: 10,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

/// In the same vein as publishers and subscriptions, services are reliable. It is especially 
/// important for services to use volatile durability, as otherwise service servers that 
/// re-start may receive outdated requests. While the client is protected from receiving 
/// multiple responses, the server is not protected from side-effects of receiving the 
/// outdated requests.
pub const QOS_PROFILE_SERVICES_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepLast,
    depth: 10,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const QOS_PROFILE_PARAMETER_EVENTS: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::KeepAll,
    depth: 1000,
    reliability: QoSReliabilityPolicy::Reliable,
    durability: QoSDurabilityPolicy::Volatile,
    avoid_ros_namespace_conventions: false,
};

pub const SYSTEM_DEFAULT: isize = 0;

/// This uses the RMW implementation’s default values for all of the policies. Different 
/// RMW implementations may have different defaults.
pub const QOS_PROFILE_SYSTEM_DEFAULT: QoSProfile = QoSProfile {
    history: QoSHistoryPolicy::SystemDefault,
    depth: SYSTEM_DEFAULT,
    reliability: QoSReliabilityPolicy::SystemDefault,
    durability: QoSDurabilityPolicy::SystemDefault,
    avoid_ros_namespace_conventions: false,
};

impl From<QoSProfile> for rmw_qos_profile_t {
    fn from(qos: QoSProfile) -> Self {
        Self {
            history: qos.history.into(),
            depth: qos.depth as usize,
            reliability: qos.reliability.into(),
            durability: qos.durability.into(),
            avoid_ros_namespace_conventions: qos.avoid_ros_namespace_conventions,
            deadline: rmw_time_t { sec: 0, nsec: 0 },
            lifespan: rmw_time_t { sec: 0, nsec: 0 },
            liveliness_lease_duration: rmw_time_t { sec: 0, nsec: 0 },
            liveliness: rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
        }
    }
}

impl From<QoSHistoryPolicy> for rmw_qos_history_policy_t {
    fn from(policy: QoSHistoryPolicy) -> Self {
        match policy {
            QoSHistoryPolicy::SystemDefault => {
                rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT
            }
            QoSHistoryPolicy::KeepLast => {
                rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST
            }
            QoSHistoryPolicy::KeepAll => rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL,
        }
    }
}

impl From<QoSReliabilityPolicy> for rmw_qos_reliability_policy_t {
    fn from(policy: QoSReliabilityPolicy) -> Self {
        match policy {
            QoSReliabilityPolicy::SystemDefault => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT
            }
            QoSReliabilityPolicy::Reliable => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE
            }
            QoSReliabilityPolicy::BestEffort => {
                rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
            }
        }
    }
}

impl From<QoSDurabilityPolicy> for rmw_qos_durability_policy_t {
    fn from(policy: QoSDurabilityPolicy) -> Self {
        match policy {
            QoSDurabilityPolicy::SystemDefault => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT
            }
            QoSDurabilityPolicy::TransientLocal => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
            }
            QoSDurabilityPolicy::Volatile => {
                rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE
            }
        }
    }
}
