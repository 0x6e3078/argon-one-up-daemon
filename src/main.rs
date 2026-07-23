// Argon ONE UP Laptop Daemon
// V6.6 - Fixed CW2217B registers adresses
// License: GPL-3.0

use rppal::gpio::{Gpio, Trigger};
use rppal::i2c::I2c;
use std::env;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;
use zbus::{interface, connection::Builder};


// Hardware constants for the Argon ONE UP (Laptop Model)
const R_SENSE: f64 = 10.0;
const ENERGY_FULL_WH: f64 = 55.21;
const PIN_LID: u8 = 27;

const REG_PROFILE_START: u8 = 0x10;
const PROFILE_SIZE: usize = 80;

const PROFILE_DATALIST: [u8; 80] = [
    0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xA8,0xAA,0xBE,0xC6,0xB8,0xAE,0xC2,0x98,
    0x82,0xFF,0xFF,0xCA,0x98,0x75,0x63,0x55,0x4E,0x4C,0x49,0x98,0x88,0xDC,0x34,0xDB,
    0xD3,0xD4,0xD3,0xD0,0xCE,0xCB,0xBB,0xE7,0xA2,0xC2,0xC4,0xAE,0x96,0x89,0x80,0x74,
    0x67,0x63,0x71,0x8E,0x9F,0x85,0x6F,0x3B,0x20,0x00,0xAB,0x10,0xFF,0xB0,0x73,0x00,
    0x00,0x00,0x64,0x08,0xD3,0x77,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFA
];

// IC: CellWise CW2217B (CW2217BAAD)
const ADDR_BATTERY: u8 = 0x64;

const REG_VCELL_H: u8 = 0x02;
const REG_VCELL_L: u8 = 0x03;
const REG_SOC_H: u8 = 0x04;
const REG_SOC_L: u8 = 0x05;

const REG_TEMP: u8 = 0x06;

const REG_CONTROL: u8 = 0x08;

const REG_SOCALERT: u8 = 0x0B;
const REG_CURRENT_H: u8 = 0x0E;
const REG_CURRENT_L: u8 = 0x0F;
const REG_ICSTATE: u8 = 0xA7;

const PIN_SHUTDOWN: u8 = 4;

#[derive(Clone, Copy, Debug, PartialEq)]
struct BatteryState {
    soc: f64,
    voltage: f64,
    current: f64,
    temperature: f64,
    power: f64,
    state: u32, // 1=Charging, 2=Discharging, 4=Full
    ac_present: bool,
    lid_closed: bool,
}

// --- UPower Manager Interface ---
struct UPowerManager {
    state: Arc<RwLock<BatteryState>>,
}

#[interface(name = "org.freedesktop.UPower")]
impl UPowerManager {
    fn enumerate_devices(&self) -> Vec<zbus::zvariant::OwnedObjectPath> {
        vec![
            zbus::zvariant::ObjectPath::from_static_str(
                "/org/freedesktop/UPower/devices/battery_argon",
            )
            .unwrap()
            .into(),
        ]
    }

    fn get_display_device(&self) -> zbus::zvariant::OwnedObjectPath {
        zbus::zvariant::ObjectPath::from_static_str("/org/freedesktop/UPower/devices/battery_argon")
            .unwrap()
            .into()
    }

    #[zbus(property)]
    fn native_path(&self) -> String {
        "argon-one-up".to_string()
    }

    #[zbus(property)]
    fn update_time(&self) -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs()
    }

    #[zbus(property)]
    fn energy_empty(&self) -> f64 {
        0.0
    }

    #[zbus(property)]
    fn capacity(&self) -> f64 {
        100.0
    }

    #[zbus(property)]
    fn time_to_empty(&self) -> i64 {
        0
    }

    #[zbus(property)]
    fn time_to_full(&self) -> i64 {
        0
    }

    #[zbus(property)]
    fn has_history(&self) -> bool {
        false
    }

    #[zbus(property)]
    fn has_statistics(&self) -> bool {
        false
    }

    #[zbus(property)]
    fn daemon_version(&self) -> String {
        "0.99.11".to_string()
    }

    #[zbus(property)]
    fn on_battery(&self) -> bool {
        !self.state.read().unwrap().ac_present
    }

    #[zbus(property)]
    fn lid_is_closed(&self) -> bool {
        self.state.read().unwrap().lid_closed
    }

    #[zbus(property)]
    fn lid_is_present(&self) -> bool {
        true
    }

    #[zbus(property)]
    fn critical_action(&self) -> String {
        "PowerOff".to_string()
    }
}

// --- UPower Device Interface ---
struct ArgonBattery {
    state: Arc<RwLock<BatteryState>>,
}

#[interface(name = "org.freedesktop.UPower.Device")]
impl ArgonBattery {
    #[zbus(property)]
    fn percentage(&self) -> f64 {
        self.state.read().unwrap().soc
    }

    #[zbus(property)]
    fn voltage(&self) -> f64 {
        self.state.read().unwrap().voltage
    }

    #[zbus(property)]
    fn energy(&self) -> f64 {
        self.state.read().unwrap().soc / 100.0 * ENERGY_FULL_WH
    }

    #[zbus(property)]
    fn energy_full(&self) -> f64 {
        ENERGY_FULL_WH
    }

    #[zbus(property)]
    fn energy_full_design(&self) -> f64 {
        ENERGY_FULL_WH
    }

    #[zbus(property)]
    fn energy_rate(&self) -> f64 {
        self.state.read().unwrap().power
    }

    #[zbus(property)]
    fn state(&self) -> u32 {
        self.state.read().unwrap().state
    }

    #[zbus(property)]
    fn is_present(&self) -> bool {
        true
    }

    #[zbus(property)]
    fn is_rechargeable(&self) -> bool {
        true
    }

    #[zbus(property)]
    fn power_supply(&self) -> bool {
        true
    }

    #[zbus(property)]
    fn technology(&self) -> u32 {
        1
    } // 1 = Li-ion

    #[zbus(property)]
    fn model(&self) -> String {
        "Argon ONE UP".to_string()
    }

    #[zbus(property)]
    fn vendor(&self) -> String {
        "Argon40".to_string()
    }

    #[zbus(property)]
    fn type_(&self) -> u32 {
        2
    } // 2 = Battery
}

struct HardwareManager {
    i2c: I2c,
    debug: bool,
    ac_history: Vec<bool>,
}

impl HardwareManager {
    fn new(debug: bool) -> Self {
        let mut i2c = I2c::with_bus(1).expect("Could not open I2C Bus 1");
        i2c.set_slave_address(ADDR_BATTERY as u16)
            .expect("Could not set I2C slave address");
        HardwareManager {
            i2c,
            debug,
            ac_history: Vec::new(),
        }
    }

    fn init(&mut self) -> bool {
        if self.debug {
            println!("[DEBUG] Initiating CW2217B controller activation...");
        }

        let mut retries = 3;

        while retries > 0 {
            retries -= 1;

            if let Err(err) = self.i2c.write(&[REG_CONTROL, 0x30]) {
                eprintln!("[ERROR] Failed to write CW2217B sleep command: {}", err);
                continue;
            }

            thread::sleep(Duration::from_millis(500));

            if let Err(err) = self.i2c.write(&[REG_CONTROL, 0x00]) {
                eprintln!("[ERROR] Failed to write CW2217B wake command: {}", err);
                continue;
            }

            thread::sleep(Duration::from_millis(500));

            let mut wait_secs = 5;

            while wait_secs > 0 {
                if let Some(status) = self.read_byte(REG_ICSTATE) {
                    if status != 255 && status != 0 {
                        if self.debug {
                            println!("[DEBUG] CW2217B Active. State: 0x{:02X}", status);
                        }
                        return true;
                    }
                }

                thread::sleep(Duration::from_secs(1));
                wait_secs -= 1;
            }
        }

        false
    }

    fn check_battery_profile_loaded(&mut self) -> bool {
        match self.read_byte(REG_SOCALERT) {
            Some(soc_alert) => {
                let profile_loaded = (soc_alert & 0x80) != 0;
                println!(
                    "battery-profile SOC_ALERT=0x{:02X} UPDATE_FLAG={}",
                    soc_alert,
                    if profile_loaded { "SET (profile loaded)" } else { "CLEAR (no profile)" }
                );
                profile_loaded
            }
            None => {
                println!("battery-profile Unable to read SOC_ALERT register");
                false
            }
        }
    }

    fn read_byte(&mut self, reg: u8) -> Option<u8> {
        match self.i2c.smbus_read_byte(reg) {
            Ok(value) => Some(value),
            Err(err) => {
                if self.debug {
                    eprintln!("[ERROR] I2C read failed at register 0x{:02X}: {}", reg, err);
                }
                None
            }
        }
    }

    fn get_status(&mut self) -> u8 {
        let stat = match self.read_byte(REG_CONTROL) {
            Some(value) => value,
            None => {
                println!("battery-status Unable to read REG_CONTROL");
                return 1;
            }
        };

        if stat != 0 {
            println!("battery-status Inactive 0x{:02X}", stat);
            return 2;
        }

        let soc_alert = match self.read_byte(REG_SOCALERT) {
            Some(value) => value,
            None => {
                println!("battery-status Unable to read REG_SOCALERT");
                return 1;
            }
        };

        if soc_alert & 0x80 == 0 {
            println!("battery-status Profile not ready 0x{:02X}", soc_alert);
            return 3;
        }

        0
    }

    fn update_status(&mut self, lid_is_low: bool) -> Option<BatteryState> {
        let v_raw_high = self.read_byte(REG_VCELL_H)?;
        let v_raw_low = self.read_byte(REG_VCELL_L)?;
        let soc_raw_high = self.read_byte(REG_SOC_H)?;
        let soc_raw_low = self.read_byte(REG_SOC_L)?;
        let temperature_raw = self.read_byte(REG_TEMP)?;
        let current_raw_high = self.read_byte(REG_CURRENT_H)?;
        let current_raw_low = self.read_byte(REG_CURRENT_L)?;

        if (v_raw_high == 255 || v_raw_high == 0) && (soc_raw_high == 255 || soc_raw_high == 0) {
            return None;
        }

        // Voltage: Bits [13:6] from VCELL_H + VCELL_L as a 14-bit value
        // LSB = 312.5µV → V(uV) = Value * 312.5
        let voltage_raw = (((v_raw_high & 0x3F) as u16) << 8) | v_raw_low as u16;
        let voltage = voltage_raw as f64 * 0.0003125;

        // SOC: 16-bit unsigned, H = integer in 1% steps, L = fractional (LSB = 1/256%)
        let soc_raw = soc_raw_high as f64 + soc_raw_low as f64 / 256.0;

        // Temperature: LSB = 0.5°C, Offset = -40°C
        // CORRECT FORMULA according to the CW2217B datasheet: TEMP(°C) = -40 + Value/2
        let temperature = temperature_raw as f64 / 2.0 - 40.0;

        // Current: SIGNED 16-bit, Two's Complement
        // I(A) = 52.4 * Value / (32768 * R_sense_mΩ)
        let raw_current = (((current_raw_high as u16) << 8) | current_raw_low as u16) as i16;
        let current = (52.4 * raw_current as f64) / (32768.0 * R_SENSE);

        let power = (voltage * current).abs();

        // Debug output of all raw values
        if self.debug {
            println!(
                "[DEBUG] RAW REGS: V=0x{:02X}:0x{:02X} SOC=0x{:02X}:0x{:02X} T=0x{:02X} I=0x{:02X}:0x{:02X}",
                v_raw_high, v_raw_low,
                soc_raw_high, soc_raw_low,
                temperature_raw,
                current_raw_high, current_raw_low
            );
            println!(
                "[DEBUG] CALC: V={:.3}V SOC={:.1}% T={:.1}°C I={:.3}A P={:.2}W",
                voltage,
                soc_raw,
                temperature,
                current,
                power
            );
        }

        // Plausibility check
        if !(2.5..=4.5).contains(&voltage) {
            eprintln!(
                "[WARN] Dropping update: voltage {:.3} V is out of range (raw=0x{:04X})",
                voltage,
                voltage_raw
            );
            return None;
        }

        // Profile check: Invalid SOC > 100% indicates a missing battery profile
        if soc_raw > 100.0 {
            eprintln!(
                "[WARN] SOC {:.1}% > 100 — likely no battery profile loaded. Clamping to 100.",
                soc_raw
            );
            // Profile invalid → SOC invalid, but still return so the GUI does not completely
            // freeze. Set state to Unknown so it is clear the value is not
            // reliable.
            return Some(BatteryState {
                soc: 100.0,
                voltage,
                current,
                temperature,
                power,
                state: 0, // Unknown
                ac_present: current > 0.0,
                lid_closed: lid_is_low,
            });
        }

        if !(0.0..=100.0).contains(&soc_raw) {
            eprintln!("[WARN] Ignoring implausible SOC: {:.2}%", soc_raw);
            return None;
        }

        if !(-40.0..=100.0).contains(&temperature) {
            eprintln!("[WARN] Ignoring implausible temperature: {:.1}°C", temperature);
            return None;
        }

        let soc = soc_raw;

        let raw_ac = current > 0.0;

        self.ac_history.push(raw_ac);
        if self.ac_history.len() > 3 {
            self.ac_history.remove(0);
        }
        let ac_present = self.ac_history.iter().filter(|&&x| x).count() >= 2;

        let state = if !ac_present {
            2 // Discharging
        } else if current > 0.05 {
            1 // Charging
        } else if soc >= 98.0 {
            4 // Full
        } else {
            1 // Charging fallback
        };

        Some(BatteryState {
            soc,
            voltage,
            current,
            temperature,
            power,
            state,
            ac_present,
            lid_closed: lid_is_low,
        })
    }

    const REG_PROFILE_START: u8 = 0x10;
    const PROFILE_SIZE: usize = 80;

    fn load_battery_profile(&mut self) -> bool {
        println!("[INFO] Loading CW2217B battery profile...");

        // Chip in Sleep
        if let Err(err) = self.i2c.write(&[REG_CONTROL, 0x30]) {
            eprintln!("[ERROR] Failed to put CW2217B into sleep: {}", err);
            return false;
        }

        thread::sleep(Duration::from_millis(100));

        // write profile data to the chip
        let mut data = Vec::with_capacity(PROFILE_SIZE + 1);
        data.push(REG_PROFILE_START);
        data.extend_from_slice(&PROFILE_DATALIST);

        if let Err(err) = self.i2c.write(&data) {
            eprintln!("[ERROR] Failed to write battery profile: {}", err);
            return false;
        }
        if self.debug {
            println!(
                "[DEBUG] Wrote {} profile bytes starting at register 0x{:02X}",
                PROFILE_DATALIST.len(),
                REG_PROFILE_START
            );
        }
        // read back and verify the profile
        if !self.verify_battery_profile() {
            eprintln!("[ERROR] Battery profile verification failed.");
            return false;
        }

        // Set UPDATE_FLAG
        if let Err(err) = self.i2c.write(&[REG_SOCALERT, 0xB0]) {
            eprintln!("[ERROR] Failed to set UPDATE_FLAG: {}", err);
            return false;
        }

        if self.debug {
            println!("[INFO] Set UPDATE_FLAG (0xB0 -> SOC_ALERT)");
        }

        thread::sleep(Duration::from_millis(100));

        // wake up controller
        if let Err(err) = self.i2c.write(&[REG_CONTROL, 0x00]) {
            eprintln!("[ERROR] Failed to wake CW2217B: {}", err);
            return false;
        }

        thread::sleep(Duration::from_millis(1500));

        let soc_alert = match self.read_byte(REG_SOCALERT) {
            Some(value) => value,
            None => {
                eprintln!("[ERROR] Could not read SOC_ALERT after profile load");
                return false;
            }
        };

        let flag_set = (soc_alert & 0x80) != 0;

        println!(
            "[INFO] Profile load verify: SOC_ALERT=0x{:02X} UPDATE_FLAG={}",
            soc_alert,
            if flag_set { "SET ✓" } else { "NOT SET ✗" }
        );

        if !flag_set {
            eprintln!("[ERROR] UPDATE_FLAG not set after profile load.");
            return false;
        }

        if self.debug {
            println!("[INFO] Battery profile loaded successfully!");
        }
        true
    }

    fn verify_battery_profile(&mut self) -> bool {
        if self.debug {
            println!("[INFO] Verifying CW2217B battery profile...");
        }

        for (i, expected) in PROFILE_DATALIST.iter().enumerate() {
            let reg = REG_PROFILE_START + i as u8;

            let actual = match self.read_byte(reg) {
                Some(value) => value,
                None => {
                    eprintln!("[ERROR] Failed to read profile byte at 0x{:02X}", reg);
                    return false;
                }
            };

            if actual != *expected {
                eprintln!(
                    "[ERROR] Profile mismatch at reg 0x{:02X}: expected 0x{:02X}, got 0x{:02X}",
                    reg,
                    expected,
                    actual
                );
                return false;
            }
        }

        if self.debug {
            println!("[INFO] Battery profile verified successfully.");
        }
        true
    }

}

#[tokio::main]
async fn main() -> zbus::Result<()> {
    let args: Vec<String> = env::args().collect();
    let debug_mode = args.iter().any(|arg| arg == "--debug" || arg == "-d");

    println!("--- Argon ONE UP Rust Manager (V6.6) ---");

    let gpio = Gpio::new().expect("GPIO Error");
    let mut hw = HardwareManager::new(debug_mode);

    thread::sleep(Duration::from_millis(500));
    if hw.get_status() != 0 {
        if !hw.init() {
            println!("[WARNING] Hardware activation sequence timed out.");
        };
    }

    // NEW: load battery profile
    if !hw.load_battery_profile() {
        println!("[ERROR] Failed to load battery profile. SOC will be unreliable!");
    }

    // NEW: check profile status
    let profile_ok = hw.check_battery_profile_loaded();

    if !profile_ok {
        println!(
            "[WARN] No battery profile loaded! SOC values will be invalid."
        );
        println!(
            "[WARN] The CW2217B requires a battery characterization profile."
        );
        println!(
            "[WARN] Contact Argon / Cellwise for the profile data for this battery."
        );
    }

    let lid_pin = gpio.get(PIN_LID).unwrap().into_input_pullup();

    let initial_state = loop {
        if let Some(state) = hw.update_status(lid_pin.is_low()) {
            println!("Battery controller synchronized.");
            break state;
        }
        // hw.init();
        thread::sleep(Duration::from_secs(2));
    };

    let shared_state = Arc::new(RwLock::new(initial_state));
    let last_broadcast_state = Arc::new(RwLock::new(initial_state));

    let battery_obj = ArgonBattery {
        state: Arc::clone(&shared_state),
    };
    let manager_obj = UPowerManager {
        state: Arc::clone(&shared_state),
    };

    let conn = Builder::system()?
        .name("org.freedesktop.UPower")?
        .serve_at("/org/freedesktop/UPower", manager_obj)?
        .serve_at("/org/freedesktop/UPower/devices/battery_argon", battery_obj)?
        .build()
        .await?;

    let object_server = conn.object_server();
    let battery_iface = object_server
        .interface::<_, ArgonBattery>("/org/freedesktop/UPower/devices/battery_argon")
        .await?;
    let manager_iface = object_server
        .interface::<_, UPowerManager>("/org/freedesktop/UPower")
        .await?;

    thread::spawn(move || {
        let gpio_btn = Gpio::new().expect("GPIO Error");
        let mut shutdown_pin = gpio_btn.get(PIN_SHUTDOWN).unwrap().into_input_pullup();
        let _ = shutdown_pin.set_interrupt(Trigger::FallingEdge, Some(Duration::from_millis(10)));
        loop {
            if let Ok(Some(_)) = shutdown_pin.poll_interrupt(true, Some(Duration::from_millis(500)))
            {
                if shutdown_pin.is_low() {
                    let mut dur = 0;
                    while shutdown_pin.is_low() && dur < 50 {
                        dur += 1;
                        thread::sleep(Duration::from_millis(100));
                    }
                    if dur >= 30 {
                        println!("Soft-shutdown signal detected!");
                    }
                }
            }
        }
    });

    loop {
        tokio::time::sleep(Duration::from_secs(2)).await;

        if let Some(new_state) = hw.update_status(lid_pin.is_low()) {
            let mut changed = false;
            {
                let mut w = shared_state.write().unwrap();
                let last = last_broadcast_state.read().unwrap();

                if new_state.state != last.state
                    || new_state.ac_present != last.ac_present
                    || new_state.lid_closed != last.lid_closed
                    || (new_state.soc - last.soc).abs() > 0.5
                    || (new_state.power - last.power).abs() > 0.05
                {
                    changed = true;
                }
                *w = new_state;
            }

            if changed {
                let mut last_w = last_broadcast_state.write().unwrap();
                *last_w = new_state;

                let bat_ctx = battery_iface.signal_context();
                let bat_inst = battery_iface.get().await;
                let _ = bat_inst.percentage_changed(bat_ctx).await;
                let _ = bat_inst.state_changed(bat_ctx).await;
                let _ = bat_inst.voltage_changed(bat_ctx).await;
                let _ = bat_inst.energy_changed(bat_ctx).await;
                let _ = bat_inst.energy_rate_changed(bat_ctx).await;

                let mgr_ctx = manager_iface.signal_context();
                let mgr_inst = manager_iface.get().await;
                let _ = mgr_inst.on_battery_changed(mgr_ctx).await;
                let _ = mgr_inst.lid_is_closed_changed(mgr_ctx).await;

                println!(
                    "[EVENT] SOC: {}% | P: {:.2}W | AC: {} | State: {}",
                    new_state.soc,
                    new_state.power,
                    if new_state.ac_present { "YES" } else { "NO" },
                    match new_state.state {
                        1 => "Charging",
                        2 => "Discharging",
                        4 => "Full",
                        _ => "Unknown",
                    }
                );
            }
        } else {
            hw.init();
        }
    }
}
