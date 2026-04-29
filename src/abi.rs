use std::ffi::{c_char, c_void, CStr, CString};

use anyhow::{anyhow, Context, Result};
use libloading::Library;

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct AbiMotorState {
    has_value: i32,
    can_id: u8,
    arbitration_id: u32,
    status_code: u8,
    pos: f32,
    vel: f32,
    torq: f32,
    t_mos: f32,
    t_rotor: f32,
}

pub struct MotorAbi {
    _lib: Library,
    motor_last_error_message: unsafe extern "C" fn() -> *const c_char,
    motor_controller_new_socketcan: unsafe extern "C" fn(*const c_char) -> *mut c_void,
    motor_controller_free: unsafe extern "C" fn(*mut c_void),
    motor_controller_poll_feedback_once: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_controller_shutdown: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_controller_add_damiao_motor: unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void,
    motor_handle_free: unsafe extern "C" fn(*mut c_void),
    motor_handle_enable: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_handle_disable: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_handle_ensure_mode: unsafe extern "C" fn(*mut c_void, u32, u32) -> i32,
    motor_handle_send_mit: unsafe extern "C" fn(*mut c_void, f32, f32, f32, f32, f32) -> i32,
    motor_handle_send_pos_vel: unsafe extern "C" fn(*mut c_void, f32, f32) -> i32,
    motor_handle_send_vel: unsafe extern "C" fn(*mut c_void, f32) -> i32,
    motor_handle_send_force_pos: unsafe extern "C" fn(*mut c_void, f32, f32, f32) -> i32,
    motor_handle_request_feedback: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_handle_get_state: unsafe extern "C" fn(*mut c_void, *mut AbiMotorState) -> i32,
}

pub struct AbiController {
    raw: *mut c_void,
}

pub struct AbiMotor {
    raw: *mut c_void,
}

#[derive(Clone, Copy)]
pub struct MotorFeedback {
    pub status_code: u8,
    pub pos: f32,
    pub vel: f32,
    pub torq: f32,
    pub t_mos: f32,
    pub t_rotor: f32,
}

impl MotorAbi {
    pub fn load_default() -> Result<Self> {
        let default_path = if cfg!(target_os = "windows") {
            "../motorbridge/target/release/motor_abi.dll"
        } else if cfg!(target_os = "macos") {
            "../motorbridge/target/release/libmotor_abi.dylib"
        } else {
            "../motorbridge/target/release/libmotor_abi.so"
        };
        let default_from_build = option_env!("MOTORBRIDGE_ABI_DEFAULT_PATH").unwrap_or(default_path);
        let path = std::env::var("MOTORBRIDGE_ABI_PATH").unwrap_or_else(|_| default_from_build.to_string());
        // SAFETY: path is controlled and symbols are validated below.
        let lib = unsafe { Library::new(&path) }.with_context(|| format!("load motor ABI library failed: {path}"))?;

        unsafe {
            macro_rules! sym {
                ($name:literal, $t:ty) => {{
                    *lib.get::<$t>($name)
                        .with_context(|| format!("missing ABI symbol: {}", String::from_utf8_lossy($name)))?
                }};
            }

            let abi = Self {
                motor_last_error_message: sym!(b"motor_last_error_message\0", unsafe extern "C" fn() -> *const c_char),
                motor_controller_new_socketcan: sym!(b"motor_controller_new_socketcan\0", unsafe extern "C" fn(*const c_char) -> *mut c_void),
                motor_controller_free: sym!(b"motor_controller_free\0", unsafe extern "C" fn(*mut c_void)),
                motor_controller_poll_feedback_once: sym!(b"motor_controller_poll_feedback_once\0", unsafe extern "C" fn(*mut c_void) -> i32),
                motor_controller_shutdown: sym!(b"motor_controller_shutdown\0", unsafe extern "C" fn(*mut c_void) -> i32),
                motor_controller_add_damiao_motor: sym!(b"motor_controller_add_damiao_motor\0", unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void),
                motor_handle_free: sym!(b"motor_handle_free\0", unsafe extern "C" fn(*mut c_void)),
                motor_handle_enable: sym!(b"motor_handle_enable\0", unsafe extern "C" fn(*mut c_void) -> i32),
                motor_handle_disable: sym!(b"motor_handle_disable\0", unsafe extern "C" fn(*mut c_void) -> i32),
                motor_handle_ensure_mode: sym!(b"motor_handle_ensure_mode\0", unsafe extern "C" fn(*mut c_void, u32, u32) -> i32),
                motor_handle_send_mit: sym!(b"motor_handle_send_mit\0", unsafe extern "C" fn(*mut c_void, f32, f32, f32, f32, f32) -> i32),
                motor_handle_send_pos_vel: sym!(b"motor_handle_send_pos_vel\0", unsafe extern "C" fn(*mut c_void, f32, f32) -> i32),
                motor_handle_send_vel: sym!(b"motor_handle_send_vel\0", unsafe extern "C" fn(*mut c_void, f32) -> i32),
                motor_handle_send_force_pos: sym!(b"motor_handle_send_force_pos\0", unsafe extern "C" fn(*mut c_void, f32, f32, f32) -> i32),
                motor_handle_request_feedback: sym!(b"motor_handle_request_feedback\0", unsafe extern "C" fn(*mut c_void) -> i32),
                motor_handle_get_state: sym!(b"motor_handle_get_state\0", unsafe extern "C" fn(*mut c_void, *mut AbiMotorState) -> i32),
                _lib: lib,
            };
            Ok(abi)
        }
    }

    pub fn new_controller_socketcan(&self, channel: &str) -> Result<AbiController> {
        let c = CString::new(channel)?;
        let raw = unsafe { (self.motor_controller_new_socketcan)(c.as_ptr()) };
        if raw.is_null() {
            return Err(anyhow!(self.last_error()));
        }
        Ok(AbiController { raw })
    }

    pub fn add_damiao_motor(&self, controller: &AbiController, motor_id: u16, feedback_id: u16, model: &str) -> Result<AbiMotor> {
        let m = CString::new(model)?;
        let raw = unsafe { (self.motor_controller_add_damiao_motor)(controller.raw, motor_id, feedback_id, m.as_ptr()) };
        if raw.is_null() {
            return Err(anyhow!(self.last_error()));
        }
        Ok(AbiMotor { raw })
    }

    pub fn controller_poll_feedback_once(&self, controller: &AbiController) -> Result<()> {
        self.rc(unsafe { (self.motor_controller_poll_feedback_once)(controller.raw) })
    }

    pub fn controller_shutdown(&self, controller: &AbiController) -> Result<()> {
        self.rc(unsafe { (self.motor_controller_shutdown)(controller.raw) })
    }

    pub fn motor_enable(&self, motor: &AbiMotor) -> Result<()> { self.rc(unsafe { (self.motor_handle_enable)(motor.raw) }) }
    pub fn motor_disable(&self, motor: &AbiMotor) -> Result<()> { self.rc(unsafe { (self.motor_handle_disable)(motor.raw) }) }
    pub fn motor_ensure_mode(&self, motor: &AbiMotor, mode: u32, timeout_ms: u32) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_ensure_mode)(motor.raw, mode, timeout_ms) })
    }
    pub fn motor_send_mit(&self, motor: &AbiMotor, pos: f32, vel: f32, kp: f32, kd: f32, tau: f32) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_send_mit)(motor.raw, pos, vel, kp, kd, tau) })
    }
    pub fn motor_send_pos_vel(&self, motor: &AbiMotor, pos: f32, vlim: f32) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_send_pos_vel)(motor.raw, pos, vlim) })
    }
    pub fn motor_send_vel(&self, motor: &AbiMotor, vel: f32) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_send_vel)(motor.raw, vel) })
    }
    pub fn motor_send_force_pos(&self, motor: &AbiMotor, pos: f32, vlim: f32, ratio: f32) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_send_force_pos)(motor.raw, pos, vlim, ratio) })
    }
    pub fn motor_request_feedback(&self, motor: &AbiMotor) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_request_feedback)(motor.raw) })
    }
    pub fn motor_get_state(&self, motor: &AbiMotor) -> Result<Option<MotorFeedback>> {
        let mut s = AbiMotorState::default();
        self.rc(unsafe { (self.motor_handle_get_state)(motor.raw, &mut s as *mut _) })?;
        if s.has_value == 0 {
            return Ok(None);
        }
        Ok(Some(MotorFeedback {
            status_code: s.status_code,
            pos: s.pos,
            vel: s.vel,
            torq: s.torq,
            t_mos: s.t_mos,
            t_rotor: s.t_rotor,
        }))
    }

    pub fn free_controller(&self, controller: AbiController) {
        unsafe { (self.motor_controller_free)(controller.raw) };
    }

    pub fn free_motor(&self, motor: AbiMotor) {
        unsafe { (self.motor_handle_free)(motor.raw) };
    }

    fn rc(&self, rc: i32) -> Result<()> {
        if rc == 0 { Ok(()) } else { Err(anyhow!(self.last_error())) }
    }

    fn last_error(&self) -> String {
        let ptr = unsafe { (self.motor_last_error_message)() };
        if ptr.is_null() {
            return "unknown ABI error".to_string();
        }
        unsafe { CStr::from_ptr(ptr) }.to_string_lossy().to_string()
    }
}

