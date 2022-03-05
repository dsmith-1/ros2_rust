use std::cmp::Ordering;
use std::convert::TryFrom;
use std::ffi::CStr;
use std::fmt::{self, Debug, Display};
use std::hash::{Hash, Hasher};
use std::ops::{Deref, DerefMut};

use crate::sequence::Sequence;
use crate::traits::SequenceAlloc;

/// A zero-terminated string of 8-bit characters.
///
/// The layout of this type is the same as `rosidl_runtime_c__String`. See the
/// [`Message`](crate::Message) trait for background information on this topic.
///
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::String;
/// let mut s = String::from("Grüß Gott!");
/// // Conversion back to a std::string::String is done with the ToString trait from the standard
/// // library.
/// assert_eq!(&s.to_string(), "Grüß Gott!");
/// ```
#[repr(C)]
pub struct String {
    /// Dynamic memory in this type is allocated and deallocated by C, but this is a detail that is managed by
    /// the relevant functions and trait impls.
    data: *mut libc::c_char,
    size: libc::size_t,
    capacity: libc::size_t,
}

/// A zero-terminated string of 16-bit characters.
///
/// The layout of this type is the same as `rosidl_runtime_c__U16String`. See the
/// [`Message`](crate::Message) trait for background information on this topic.
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::WString;
/// let mut s = WString::from("Grüß Gott!");
/// // Conversion back to a std::string::String is done with the ToString trait from the standard
/// // library.
/// assert_eq!(&s.to_string(), "Grüß Gott!");
/// ```
#[repr(C)]
pub struct WString {
    data: *mut libc::c_ushort,
    size: libc::size_t,
    capacity: libc::size_t,
}

/// A zero-terminated string of 8-bit characters with a length limit.
///
/// The same as [`String`], but it cannot be constructed from a string that is too large.
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::BoundedString;
/// # use std::convert::TryFrom;
/// let mut maybe_str = BoundedString::<3>::try_from("noo!");
/// assert!(maybe_str.is_err());
/// maybe_str = BoundedString::<3>::try_from("ok!");
/// assert!(maybe_str.is_ok());
/// let bounded_str = maybe_str.unwrap();
/// assert_eq!(&bounded_str.to_string(), "ok!");
/// ```
#[derive(Clone, Default, Hash, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct BoundedString<const N: usize> {
    inner: String,
}

/// A zero-terminated string of 16-bit characters with a length limit.
///
/// The same as [`WString`], but it cannot be constructed from a string that is too large.
///
/// # Example
///
/// ```
/// # use rosidl_runtime_rs::BoundedWString;
/// # use std::convert::TryFrom;
/// let mut maybe_wstr = BoundedWString::<3>::try_from("noo!");
/// assert!(maybe_wstr.is_err());
/// maybe_wstr = BoundedWString::<3>::try_from("ok!");
/// assert!(maybe_wstr.is_ok());
/// let bounded_wstr = maybe_wstr.unwrap();
/// assert_eq!(&bounded_wstr.to_string(), "ok!");
/// ```
#[derive(Clone, Default, Hash, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct BoundedWString<const N: usize> {
    inner: WString,
}

/// Error type for [`BoundedString::try_from()`] and [`BoundedWString::try_from()`].
#[derive(Debug)]
pub struct StringExceedsBoundsError {
    len: usize,
    upper_bound: usize,
}

// ========================= impls for String and WString =========================

// There is a lot of redundancy between String and WString, which this macro aims to reduce.
macro_rules! string_impl {
    ($string:ty, $char_type:ty, $string_conversion_func:ident, $init:ident, $fini:ident, $assignn:ident, $sequence_init:ident, $sequence_fini:ident, $sequence_copy:ident) => {
        #[link(name = "rosidl_runtime_c")]
        extern "C" {
            fn $init(s: *mut $string);
            fn $fini(s: *mut $string);
            fn $assignn(s: *mut $string, value: *const $char_type, n: libc::size_t);
            fn $sequence_init(seq: *mut Sequence<$string>, size: libc::size_t) -> bool;
            fn $sequence_fini(seq: *mut Sequence<$string>);
            fn $sequence_copy(
                in_seq: *const Sequence<$string>,
                out_seq: *mut Sequence<$string>,
            ) -> bool;
        }

        impl Default for $string {
            fn default() -> Self {
                let mut msg = Self {
                    data: std::ptr::null_mut(),
                    size: 0,
                    capacity: 0,
                };
                unsafe {
                    $init(&mut msg as *mut _);
                }
                msg
            }
        }

        impl Clone for $string {
            fn clone(&self) -> Self {
                let mut msg = Self {
                    data: std::ptr::null_mut(),
                    size: 0,
                    capacity: 0,
                };
                unsafe { $assignn(&mut msg as *mut _, self.data as *const _, self.size) }
                msg
            }
        }

        impl Debug for $string {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
                Debug::fmt(&self.to_string(), f)
            }
        }

        // It's not guaranteed that there are no interior null bytes, hence no Deref to CStr.
        // This does not include the null byte at the end!
        impl Deref for $string {
            type Target = [$char_type];
            fn deref(&self) -> &Self::Target {
                unsafe { std::slice::from_raw_parts(self.data as *const $char_type, self.size) }
            }
        }

        impl DerefMut for $string {
            fn deref_mut(&mut self) -> &mut Self::Target {
                unsafe { std::slice::from_raw_parts_mut(self.data as *mut $char_type, self.size) }
            }
        }

        impl Display for $string {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
                let converted = std::string::String::$string_conversion_func(self.deref());
                Display::fmt(&converted, f)
            }
        }

        impl Drop for $string {
            fn drop(&mut self) {
                unsafe {
                    $fini(self as *mut _);
                }
            }
        }

        impl Eq for $string {}

        impl Hash for $string {
            fn hash<H: Hasher>(&self, state: &mut H) {
                self.deref().hash(state)
            }
        }

        impl PartialEq for $string {
            fn eq(&self, other: &Self) -> bool {
                self.deref().eq(other.deref())
            }
        }

        impl Ord for $string {
            fn cmp(&self, other: &Self) -> Ordering {
                self.deref().cmp(other.deref())
            }
        }

        impl PartialOrd for $string {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                self.deref().partial_cmp(other.deref())
            }
        }

        impl SequenceAlloc for $string {
            fn sequence_init(seq: &mut Sequence<Self>, size: libc::size_t) -> bool {
                unsafe { $sequence_init(seq as *mut _, size) }
            }
            fn sequence_fini(seq: &mut Sequence<Self>) {
                unsafe { $sequence_fini(seq as *mut _) }
            }
            fn sequence_copy(in_seq: &Sequence<Self>, out_seq: &mut Sequence<Self>) -> bool {
                unsafe { $sequence_copy(in_seq as *const _, out_seq as *mut _) }
            }
        }
    };
}

string_impl!(
    String,
    u8,
    from_utf8_lossy,
    rosidl_runtime_c__String__init,
    rosidl_runtime_c__String__fini,
    rosidl_runtime_c__String__assignn,
    rosidl_runtime_c__String__Sequence__init,
    rosidl_runtime_c__String__Sequence__fini,
    rosidl_runtime_c__String__Sequence__copy
);
string_impl!(
    WString,
    libc::c_ushort,
    from_utf16_lossy,
    rosidl_runtime_c__U16String__init,
    rosidl_runtime_c__U16String__fini,
    rosidl_runtime_c__U16String__assignn,
    rosidl_runtime_c__U16String__Sequence__init,
    rosidl_runtime_c__U16String__Sequence__fini,
    rosidl_runtime_c__U16String__Sequence__copy
);

impl From<&str> for String {
    fn from(s: &str) -> Self {
        let mut msg = Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        unsafe {
            rosidl_runtime_c__String__assignn(&mut msg as *mut _, s.as_ptr() as *const _, s.len())
        }
        msg
    }
}

impl String {
    /// Creates a CStr from this String.
    ///
    /// This scales with the length of the string but does not create copy of the string.
    /// See also [`CStr::from_ptr()`].
    pub fn to_cstr(&self) -> &CStr {
        unsafe { CStr::from_ptr(self.data as *const _) }
    }
}

impl From<&str> for WString {
    fn from(s: &str) -> Self {
        let mut msg = Self {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        let buf: Vec<u16> = s.encode_utf16().collect();
        unsafe {
            rosidl_runtime_c__U16String__assignn(
                &mut msg as *mut _,
                buf.as_ptr() as *const _,
                buf.len(),
            )
        }
        msg
    }
}

// ========================= impl for BoundedString =========================

impl<const N: usize> Debug for BoundedString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        Debug::fmt(&self.inner, f)
    }
}

impl<const N: usize> Deref for BoundedString<N> {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        unsafe { std::slice::from_raw_parts(self.inner.data as *const u8, self.inner.size) }
    }
}

impl<const N: usize> DerefMut for BoundedString<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { std::slice::from_raw_parts_mut(self.inner.data as *mut u8, self.inner.size) }
    }
}

impl<const N: usize> Display for BoundedString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        Display::fmt(&self.inner, f)
    }
}

impl<const N: usize> Drop for BoundedString<N> {
    fn drop(&mut self) {
        unsafe { rosidl_runtime_c__String__fini(&mut self.inner as *mut _) }
    }
}

impl<const N: usize> SequenceAlloc for BoundedString<N> {
    fn sequence_init(seq: &mut Sequence<Self>, size: libc::size_t) -> bool {
        unsafe {
            rosidl_runtime_c__String__Sequence__init(seq as *mut Sequence<Self> as *mut _, size)
        }
    }
    fn sequence_fini(seq: &mut Sequence<Self>) {
        unsafe { rosidl_runtime_c__String__Sequence__fini(seq as *mut Sequence<Self> as *mut _) }
    }
    fn sequence_copy(in_seq: &Sequence<Self>, out_seq: &mut Sequence<Self>) -> bool {
        unsafe {
            rosidl_runtime_c__String__Sequence__copy(
                in_seq as *const Sequence<Self> as *const _,
                out_seq as *mut Sequence<Self> as *mut _,
            )
        }
    }
}

impl<const N: usize> TryFrom<&str> for BoundedString<N> {
    type Error = StringExceedsBoundsError;
    fn try_from(s: &str) -> Result<Self, Self::Error> {
        if s.len() <= N {
            Ok(Self {
                inner: String::from(s),
            })
        } else {
            Err(StringExceedsBoundsError {
                len: s.len(),
                upper_bound: N,
            })
        }
    }
}

// ========================= impl for BoundedWString =========================

impl<const N: usize> Debug for BoundedWString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        Debug::fmt(&self.inner, f)
    }
}

impl<const N: usize> Deref for BoundedWString<N> {
    type Target = [u16];
    fn deref(&self) -> &Self::Target {
        unsafe { std::slice::from_raw_parts(self.inner.data, self.inner.size) }
    }
}

impl<const N: usize> DerefMut for BoundedWString<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { std::slice::from_raw_parts_mut(self.inner.data, self.inner.size) }
    }
}

impl<const N: usize> Display for BoundedWString<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        Display::fmt(&self.inner, f)
    }
}

impl<const N: usize> Drop for BoundedWString<N> {
    fn drop(&mut self) {
        unsafe { rosidl_runtime_c__U16String__fini(&mut self.inner as *mut _) }
    }
}

impl<const N: usize> SequenceAlloc for BoundedWString<N> {
    fn sequence_init(seq: &mut Sequence<Self>, size: libc::size_t) -> bool {
        unsafe {
            rosidl_runtime_c__U16String__Sequence__init(seq as *mut Sequence<Self> as *mut _, size)
        }
    }
    fn sequence_fini(seq: &mut Sequence<Self>) {
        unsafe { rosidl_runtime_c__U16String__Sequence__fini(seq as *mut Sequence<Self> as *mut _) }
    }
    fn sequence_copy(in_seq: &Sequence<Self>, out_seq: &mut Sequence<Self>) -> bool {
        unsafe {
            rosidl_runtime_c__U16String__Sequence__copy(
                in_seq as *const Sequence<Self> as *const _,
                out_seq as *mut Sequence<Self> as *mut _,
            )
        }
    }
}

impl<const N: usize> TryFrom<&str> for BoundedWString<N> {
    type Error = StringExceedsBoundsError;
    fn try_from(s: &str) -> Result<Self, Self::Error> {
        if s.len() <= N {
            Ok(Self {
                inner: WString::from(s),
            })
        } else {
            Err(StringExceedsBoundsError {
                len: s.len(),
                upper_bound: N,
            })
        }
    }
}

// ========================= impl for StringExceedsBoundsError =========================

impl Display for StringExceedsBoundsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        write!(
            f,
            "BoundedString with upper bound {} initialized with len {}",
            self.upper_bound, self.len
        )
    }
}

impl std::error::Error for StringExceedsBoundsError {}
