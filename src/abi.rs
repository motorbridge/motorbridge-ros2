use std::ffi::{c_char, c_void, CStr, CString};
use std::path::{Path, PathBuf};

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
    motor_controller_new_socketcanfd: unsafe extern "C" fn(*const c_char) -> *mut c_void,
    motor_controller_new_dm_serial: unsafe extern "C" fn(*const c_char, u32) -> *mut c_void,
    motor_controller_free: unsafe extern "C" fn(*mut c_void),
    motor_controller_poll_feedback_once: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_controller_shutdown: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_controller_add_damiao_motor:
        unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void,
    motor_controller_add_hexfellow_motor:
        unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void,
    motor_controller_add_myactuator_motor:
        unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void,
    motor_controller_add_robstride_motor:
        unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void,
    motor_controller_add_hightorque_motor:
        unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void,
    motor_handle_free: unsafe extern "C" fn(*mut c_void),
    motor_handle_enable: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_handle_disable: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_handle_clear_error: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_handle_set_zero_position: unsafe extern "C" fn(*mut c_void) -> i32,
    motor_handle_ensure_mode: unsafe extern "C" fn(*mut c_void, u32, u32) -> i32,
    motor_handle_send_mit: unsafe extern "C" fn(*mut c_void, f32, f32, f32, f32, f32) -> i32,
    motor_handle_send_pos_vel: unsafe extern "C" fn(*mut c_void, f32, f32) -> i32,
    motor_handle_send_vel: unsafe extern "C" fn(*mut c_void, f32) -> i32,
    motor_handle_send_force_pos: unsafe extern "C" fn(*mut c_void, f32, f32, f32) -> i32,
    motor_handle_store_parameters: unsafe extern "C" fn(*mut c_void) -> i32,
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
        let path = resolve_abi_path()?;
        let lib = unsafe { Library::new(&path) }
            .with_context(|| format!("load motor ABI library failed: {}", path.display()))?;

        unsafe {
            macro_rules! sym {
                ($name:literal, $t:ty) => {{
                    *lib.get::<$t>($name).with_context(|| {
                        format!("missing ABI symbol: {}", String::from_utf8_lossy($name))
                    })?
                }};
            }

            Ok(Self {
                motor_last_error_message: sym!(
                    b"motor_last_error_message\0",
                    unsafe extern "C" fn() -> *const c_char
                ),
                motor_controller_new_socketcan: sym!(
                    b"motor_controller_new_socketcan\0",
                    unsafe extern "C" fn(*const c_char) -> *mut c_void
                ),
                motor_controller_new_socketcanfd: sym!(
                    b"motor_controller_new_socketcanfd\0",
                    unsafe extern "C" fn(*const c_char) -> *mut c_void
                ),
                motor_controller_new_dm_serial: sym!(
                    b"motor_controller_new_dm_serial\0",
                    unsafe extern "C" fn(*const c_char, u32) -> *mut c_void
                ),
                motor_controller_free: sym!(
                    b"motor_controller_free\0",
                    unsafe extern "C" fn(*mut c_void)
                ),
                motor_controller_poll_feedback_once: sym!(
                    b"motor_controller_poll_feedback_once\0",
                    unsafe extern "C" fn(*mut c_void) -> i32
                ),
                motor_controller_shutdown: sym!(
                    b"motor_controller_shutdown\0",
                    unsafe extern "C" fn(*mut c_void) -> i32
                ),
                motor_controller_add_damiao_motor: sym!(
                    b"motor_controller_add_damiao_motor\0",
                    unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void
                ),
                motor_controller_add_hexfellow_motor: sym!(
                    b"motor_controller_add_hexfellow_motor\0",
                    unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void
                ),
                motor_controller_add_myactuator_motor: sym!(
                    b"motor_controller_add_myactuator_motor\0",
                    unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void
                ),
                motor_controller_add_robstride_motor: sym!(
                    b"motor_controller_add_robstride_motor\0",
                    unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void
                ),
                motor_controller_add_hightorque_motor: sym!(
                    b"motor_controller_add_hightorque_motor\0",
                    unsafe extern "C" fn(*mut c_void, u16, u16, *const c_char) -> *mut c_void
                ),
                motor_handle_free: sym!(b"motor_handle_free\0", unsafe extern "C" fn(*mut c_void)),
                motor_handle_enable: sym!(
                    b"motor_handle_enable\0",
                    unsafe extern "C" fn(*mut c_void) -> i32
                ),
                motor_handle_disable: sym!(
                    b"motor_handle_disable\0",
                    unsafe extern "C" fn(*mut c_void) -> i32
                ),
                motor_handle_clear_error: sym!(
                    b"motor_handle_clear_error\0",
                    unsafe extern "C" fn(*mut c_void) -> i32
                ),
                motor_handle_set_zero_position: sym!(
                    b"motor_handle_set_zero_position\0",
                    unsafe extern "C" fn(*mut c_void) -> i32
                ),
                motor_handle_ensure_mode: sym!(
                    b"motor_handle_ensure_mode\0",
                    unsafe extern "C" fn(*mut c_void, u32, u32) -> i32
                ),
                motor_handle_send_mit: sym!(
                    b"motor_handle_send_mit\0",
                    unsafe extern "C" fn(*mut c_void, f32, f32, f32, f32, f32) -> i32
                ),
                motor_handle_send_pos_vel: sym!(
                    b"motor_handle_send_pos_vel\0",
                    unsafe extern "C" fn(*mut c_void, f32, f32) -> i32
                ),
                motor_handle_send_vel: sym!(
                    b"motor_handle_send_vel\0",
                    unsafe extern "C" fn(*mut c_void, f32) -> i32
                ),
                motor_handle_send_force_pos: sym!(
                    b"motor_handle_send_force_pos\0",
                    unsafe extern "C" fn(*mut c_void, f32, f32, f32) -> i32
                ),
                motor_handle_store_parameters: sym!(
                    b"motor_handle_store_parameters\0",
                    unsafe extern "C" fn(*mut c_void) -> i32
                ),
                motor_handle_request_feedback: sym!(
                    b"motor_handle_request_feedback\0",
                    unsafe extern "C" fn(*mut c_void) -> i32
                ),
                motor_handle_get_state: sym!(
                    b"motor_handle_get_state\0",
                    unsafe extern "C" fn(*mut c_void, *mut AbiMotorState) -> i32
                ),
                _lib: lib,
            })
        }
    }

    pub fn new_controller(
        &self,
        transport: &str,
        endpoint: &str,
        baud: Option<u32>,
    ) -> Result<AbiController> {
        let c = CString::new(endpoint)?;
        let transport = normalize_transport(transport);
        let raw = match transport.as_str() {
            "socketcan" | "can" | "auto" => unsafe {
                (self.motor_controller_new_socketcan)(c.as_ptr())
            },
            "socketcanfd" | "canfd" => unsafe {
                (self.motor_controller_new_socketcanfd)(c.as_ptr())
            },
            "dmserial" | "serial" => unsafe {
                (self.motor_controller_new_dm_serial)(c.as_ptr(), baud.unwrap_or(921_600))
            },
            other => return Err(anyhow!("unsupported motorbridge ABI transport: {other}")),
        };
        if raw.is_null() {
            return Err(anyhow!(self.last_error()));
        }
        Ok(AbiController { raw })
    }

    pub fn add_motor(
        &self,
        controller: &AbiController,
        vendor: &str,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<AbiMotor> {
        let m = CString::new(model)?;
        let raw = match normalize_vendor(vendor).as_str() {
            "damiao" => unsafe {
                (self.motor_controller_add_damiao_motor)(
                    controller.raw,
                    motor_id,
                    feedback_id,
                    m.as_ptr(),
                )
            },
            "hexfellow" => unsafe {
                (self.motor_controller_add_hexfellow_motor)(
                    controller.raw,
                    motor_id,
                    feedback_id,
                    m.as_ptr(),
                )
            },
            "myactuator" => unsafe {
                (self.motor_controller_add_myactuator_motor)(
                    controller.raw,
                    motor_id,
                    feedback_id,
                    m.as_ptr(),
                )
            },
            "robstride" => unsafe {
                (self.motor_controller_add_robstride_motor)(
                    controller.raw,
                    motor_id,
                    feedback_id,
                    m.as_ptr(),
                )
            },
            "hightorque" => unsafe {
                (self.motor_controller_add_hightorque_motor)(
                    controller.raw,
                    motor_id,
                    feedback_id,
                    m.as_ptr(),
                )
            },
            other => return Err(anyhow!("unsupported motorbridge ABI vendor: {other}")),
        };
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

    pub fn motor_enable(&self, motor: &AbiMotor) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_enable)(motor.raw) })
    }

    pub fn motor_disable(&self, motor: &AbiMotor) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_disable)(motor.raw) })
    }

    pub fn motor_clear_error(&self, motor: &AbiMotor) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_clear_error)(motor.raw) })
    }

    pub fn motor_set_zero_position(&self, motor: &AbiMotor) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_set_zero_position)(motor.raw) })
    }

    pub fn motor_ensure_mode(&self, motor: &AbiMotor, mode: u32, timeout_ms: u32) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_ensure_mode)(motor.raw, mode, timeout_ms) })
    }

    pub fn motor_send_mit(
        &self,
        motor: &AbiMotor,
        pos: f32,
        vel: f32,
        kp: f32,
        kd: f32,
        tau: f32,
    ) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_send_mit)(motor.raw, pos, vel, kp, kd, tau) })
    }

    pub fn motor_send_pos_vel(&self, motor: &AbiMotor, pos: f32, vlim: f32) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_send_pos_vel)(motor.raw, pos, vlim) })
    }

    pub fn motor_send_vel(&self, motor: &AbiMotor, vel: f32) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_send_vel)(motor.raw, vel) })
    }

    pub fn motor_send_force_pos(
        &self,
        motor: &AbiMotor,
        pos: f32,
        vlim: f32,
        ratio: f32,
    ) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_send_force_pos)(motor.raw, pos, vlim, ratio) })
    }

    pub fn motor_store_parameters(&self, motor: &AbiMotor) -> Result<()> {
        self.rc(unsafe { (self.motor_handle_store_parameters)(motor.raw) })
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
        if rc == 0 {
            Ok(())
        } else {
            Err(anyhow!(self.last_error()))
        }
    }

    fn last_error(&self) -> String {
        let ptr = unsafe { (self.motor_last_error_message)() };
        if ptr.is_null() {
            return "unknown ABI error".to_string();
        }
        unsafe { CStr::from_ptr(ptr) }.to_string_lossy().to_string()
    }
}

pub fn normalize_vendor(vendor: &str) -> String {
    vendor
        .trim()
        .to_ascii_lowercase()
        .replace(['-', '_', ' '], "")
}

pub fn normalize_transport(transport: &str) -> String {
    transport
        .trim()
        .to_ascii_lowercase()
        .replace(['-', '_', ' '], "")
}

fn abi_lib_name() -> &'static str {
    if cfg!(target_os = "windows") {
        "motor_abi.dll"
    } else if cfg!(target_os = "macos") {
        "libmotor_abi.dylib"
    } else {
        "libmotor_abi.so"
    }
}

fn resolve_abi_path() -> Result<PathBuf> {
    if let Ok(path) = std::env::var("MOTORBRIDGE_ABI_PATH") {
        return Ok(PathBuf::from(path));
    }

    let lib_name = abi_lib_name();
    let mut candidates = Vec::new();

    if let Some(path) = option_env!("MOTORBRIDGE_ABI_DEFAULT_PATH") {
        candidates.push(PathBuf::from(path));
    }

    if let Ok(exe) = std::env::current_exe() {
        if let Some(exe_dir) = exe.parent() {
            candidates.push(exe_dir.join("abi").join(lib_name));
            candidates.push(exe_dir.join(lib_name));
        }
    }

    candidates.push(Path::new("abi").join(lib_name));

    candidates.into_iter().find(|p| p.exists()).ok_or_else(|| {
        anyhow!("motor ABI artifact not found; run cargo build first or set MOTORBRIDGE_ABI_PATH")
    })
}
