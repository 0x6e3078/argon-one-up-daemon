// Argon ONE UP Laptop Daemon (Integrated Version)
// V6.5 - Fixed AC detection and CW2217B native current registers
// License: GPL-3.0

use rppal::i2c::I2c;
use rppal::gpio::{Gpio, Trigger};
use std::thread;
use std::time::Duration;
use std::sync::{Arc, RwLock};
use std::env;
use zbus::{interface, connection::Builder};

// Hardware constants for the Argon ONE UP (Laptop Model)
const R_SENSE: f64 = 10.0;
const PIN_LID: u8 = 27;

// IC: CellWise CW2217B (CW2217BAAD)
const ADDR_BATTERY: u8 = 0x64;
const REG_CONTROL: u8 = 0x01;
const REG_ICSTATE: u8 = 0x03;
const PIN_SHUTDOWN: u8 = 4;
const VCELL_H: u8 = 0x02;
const VCELL_L: u8 = 0x03;
const SOC_H: u8 = 0x04;
const SOC_L: u8 = 0x05;
const TEMP: u8 = 0x06;
const CURRENT_H: u8 = 0x0E;
const CURRENT_L: u8 = 0x0F;


#[derive(Clone, Copy, Debug, PartialEq)]
struct BatteryState {
    soc: f64,
    voltage: f64,
    current: f64,
    temperature: f64,
    power: f64,
    state: u32,     // 1=Charging, 2=Discharging, 4=Full
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
            zbus::zvariant::ObjectPath::from_static_str("/org/freedesktop/UPower/devices/battery_argon")
                .unwrap()
                .into()
        ]
    }

    fn get_display_device(&self) -> zbus::zvariant::OwnedObjectPath {
        zbus::zvariant::ObjectPath::from_static_str("/org/freedesktop/UPower/devices/battery_argon")
            .unwrap()
            .into()
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
        self.state.read().unwrap().soc * 0.5521
    }

    #[zbus(property)]
    fn energy_full(&self) -> f64 {
        55.21
    }

    #[zbus(property)]
    fn energy_full_design(&self) -> f64 {
        55.21
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
    fn is_present(&self) -> bool { true }

    #[zbus(property)]
    fn is_rechargeable(&self) -> bool { true }

    #[zbus(property)]
    fn power_supply(&self) -> bool { true }

    #[zbus(property)]
    fn technology(&self) -> u32 { 1 } // 1 = Li-ion

    #[zbus(property)]
    fn model(&self) -> String { "Argon ONE UP".to_string() }

    #[zbus(property)]
    fn vendor(&self) -> String { "Argon40".to_string() }

    #[zbus(property)]
    fn type_(&self) -> u32 { 2 } // 2 = Battery
}

struct HardwareManager {
    i2c: I2c,
    debug: bool,
    ac_history: Vec<bool>,
}

impl HardwareManager {
    fn new(debug: bool) -> Self {
        let mut i2c = I2c::with_bus(1).expect("Could not open I2C Bus 1");
        i2c.set_slave_address(ADDR_BATTERY as u16).expect("Could not set I2C slave address");
        HardwareManager { i2c, debug, ac_history: Vec::new() }
    }

    fn init(&mut self) -> bool {
        if self.debug { println!("[DEBUG] Initiating CW2217B controller activation..."); }
        let mut retries = 3;

        while retries > 0 {
            retries -= 1;
            let _ = self.i2c.write(&[REG_CONTROL, 0x30]);
            thread::sleep(Duration::from_millis(500));
            let _ = self.i2c.write(&[REG_CONTROL, 0x00]);
            thread::sleep(Duration::from_millis(500));

            let mut wait_secs = 5;
            while wait_secs > 0 {
                let status = self.read_byte(REG_ICSTATE);
                if status != 255 && status != 0 && (status & 0x0C) != 0 {
                    if self.debug { println!("[DEBUG] CW2217B Active. State: 0x{:02X}", status); }
                    return true;
                }
                thread::sleep(Duration::from_secs(1));
                wait_secs -= 1;
            }
        }
        false
    }

    fn read_byte(&mut self, reg: u8) -> u8 {
        let mut res = [0u8; 1];
        if self.i2c.write(&[reg]).is_err() { return 255; }
        thread::sleep(Duration::from_millis(20));
        if self.i2c.read(&mut res).is_err() { return 255; }
        res[0]
    }

    fn update_status(&mut self, lid_is_low: bool) -> Option<BatteryState> {


        let v_raw = self.read_byte(VCELL_H);
        let s_raw = self.read_byte(VCELL_L);
        let soc_raw_high = self.read_byte(SOC_H);
        let soc_raw_low = self.read_byte(SOC_L);
        let temperature_raw = self.read_byte(TEMP);
        let current_raw_high = self.read_byte(CURRENT_H);
        let current_raw_low = self.read_byte(CURRENT_L);

        if (v_raw == 255 || v_raw == 0) && (soc_raw_high == 255 || soc_raw_high == 0) {
            return None;
        }

        let voltage = v_raw as f64 * 0.24;
        let soc = if soc_raw_high > 100 { 100.0 } else { soc_raw_high as f64 };

        // Process Current (Signed 16-bit integer)
        let raw_current = (((current_raw_high as u16) << 8) | (current_raw_low as u16)) as i16;

        let temperature = (temperature_raw * 2) as f64 / 10.0;
        // CW2217B Current Calculation:

        let current = (52.4 * raw_current as f64 ) / (32768.0 * R_SENSE);
        // let current = raw_current as f64 / 4000.0;
        let power = (voltage * current).abs();

        // AC DETECTION LOGIC:
        let raw_ac = current > 0.0;

        // Debounce AC detection
        self.ac_history.push(raw_ac);
        if self.ac_history.len() > 3 { self.ac_history.remove(0); }
        let ac_present = self.ac_history.iter().filter(|&&x| x).count() >= 2;

        let state = if !ac_present {
            2 // Discharging
        } else if current > 0.05 {
            1 // Charging
        } else if soc >= 98.0 {
            4 // Full
        } else {
            1 // Charging Fallback
        };

        if self.debug {
            println!("[DEBUG] CW2217B -> V: {:.2}V | I: {:.3}A | P: {:.2}W | SOC: {:.1}% | Temperature: {:.4} | AC: {} | Lid: {}",
                     voltage, current, power, soc, temperature,
                     if ac_present { "YES" } else { "NO" },
                     if lid_is_low { "CLOSED" } else { "OPEN" });
        }

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
}

#[tokio::main]
async fn main() -> zbus::Result<()> {
    let args: Vec<String> = env::args().collect();
    let debug_mode = args.iter().any(|arg| arg == "--debug" || arg == "-d");

    println!("--- Argon ONE UP Rust Manager (V6.4 Stable) ---");

    let gpio = Gpio::new().expect("GPIO Error");
    let mut hw = HardwareManager::new(debug_mode);

    thread::sleep(Duration::from_millis(500));
    if !hw.init() {
        println!("[WARNING] Hardware activation sequence timed out.");
    }

    let lid_pin = gpio.get(PIN_LID).unwrap().into_input_pullup();

    let initial_state = loop {
        if let Some(state) = hw.update_status(lid_pin.is_low()) {
            println!("Battery controller synchronized.");
            break state;
        }
        hw.init();
        thread::sleep(Duration::from_secs(2));
    };

    let shared_state = Arc::new(RwLock::new(initial_state));
    let last_broadcast_state = Arc::new(RwLock::new(initial_state));

    let battery_obj = ArgonBattery { state: Arc::clone(&shared_state) };
    let manager_obj = UPowerManager { state: Arc::clone(&shared_state) };

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
            if let Ok(Some(_)) = shutdown_pin.poll_interrupt(true, Some(Duration::from_millis(500))) {
                if shutdown_pin.is_low() {
                    let mut dur = 0;
                    while shutdown_pin.is_low() && dur < 50 {
                        dur += 1;
                        thread::sleep(Duration::from_millis(100));
                    }
                    if dur >= 30 { println!("Soft-shutdown signal detected!"); }
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

                if new_state.state != last.state ||
                   new_state.ac_present != last.ac_present ||
                   new_state.lid_closed != last.lid_closed ||
                   (new_state.soc - last.soc).abs() > 0.5 ||
                   (new_state.power - last.power).abs() > 0.05
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

                println!("[EVENT] SOC: {}% | P: {:.2}W | AC: {} | State: {}",
                    new_state.soc,
                    new_state.power,
                    if new_state.ac_present { "YES" } else { "NO" },
                    match new_state.state { 1 => "Charging", 2 => "Discharging", 4 => "Full", _ => "Unknown" }
                );
            }
        } else {
            hw.init();
        }
    }
}