// Argon ONE UP Laptop Daemon (Integrated Version)
// V5.3 - Extended Daemon Info (Lid, Critical Action) and D-Bus Signaling
// License: GPL-3.0

use rppal::i2c::I2c;
use rppal::gpio::{Gpio, Trigger};
use std::thread;
use std::time::Duration;
use std::sync::{Arc, RwLock};
use zbus::{interface, connection::Builder};

// Hardware constants for the Laptop model
const ADDR_BATTERY: u8 = 0x64;
const PIN_SHUTDOWN: u8 = 4;
const PIN_LID: u8 = 27;

#[derive(Clone, Copy, Debug, PartialEq)]
struct BatteryState {
    capacity: f64,
    voltage: f64,
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
        self.state.read().unwrap().capacity
    }

    #[zbus(property)]
    fn voltage(&self) -> f64 {
        self.state.read().unwrap().voltage
    }

    #[zbus(property)]
    fn energy(&self) -> f64 {
        // Based on 55.21 Wh battery (4780 mAh)
        self.state.read().unwrap().capacity * 0.5521
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
}

impl HardwareManager {
    fn new() -> Self {
        let i2c = I2c::with_bus(1).expect("Could not open I2C Bus 1");
        HardwareManager { i2c }
    }

    fn read_byte(&mut self, reg: u8) -> u8 {
        let _ = self.i2c.set_slave_address(ADDR_BATTERY as u16);
        let _ = self.i2c.write(&[reg]);
        thread::sleep(Duration::from_millis(15));
        let mut res = [0u8; 1];
        if self.i2c.read(&mut res).is_err() {
            return 255;
        }
        res[0]
    }

    fn update_status(&mut self, lid_is_low: bool) -> Option<BatteryState> {
        let regs: Vec<u8> = (0..6).map(|r| self.read_byte(r)).collect();

        let v_raw = regs[2];
        let s_raw = regs[3];
        let c_raw = regs[4];

        if v_raw == 255 || s_raw == 255 || c_raw == 255 { return None; }

        let voltage = v_raw as f64 * 0.24;
        let ac_present = voltage > 13.0;
        let capacity = if c_raw > 100 { 100.0 } else { c_raw as f64 };

        let state = if !ac_present {
            2 // Discharging
        } else if capacity >= 100.0 {
            4 // Full
        } else {
            1 // Charging
        };

        Some(BatteryState {
            capacity,
            voltage,
            state,
            ac_present,
            lid_closed: lid_is_low,
        })
    }
}

#[tokio::main]
async fn main() -> zbus::Result<()> {
    println!("--- Argon ONE UP Rust Manager (V5.3 International) ---");

    let gpio = Gpio::new().expect("GPIO Error");
    let mut hw = HardwareManager::new();
    let lid_pin = gpio.get(PIN_LID).unwrap().into_input_pullup();

    println!("Initializing hardware status...");
    let initial_state = loop {
        if let Some(state) = hw.update_status(lid_pin.is_low()) {
            break state;
        }
        thread::sleep(Duration::from_secs(1));
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

    // GPIO thread for Shutdown Button
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
                    if dur >= 30 { println!("Shutdown triggered!"); }
                }
            }
        }
    });

    // Main loop for hardware polling and D-Bus signals
    loop {
        tokio::time::sleep(Duration::from_secs(5)).await;

        if let Some(new_state) = hw.update_status(lid_pin.is_low()) {
            let mut changed = false;
            {
                let mut w = shared_state.write().unwrap();
                let last = last_broadcast_state.read().unwrap();

                if new_state.state != last.state ||
                   new_state.ac_present != last.ac_present ||
                   new_state.lid_closed != last.lid_closed ||
                   (new_state.capacity - last.capacity).abs() > 0.9 ||
                   (new_state.voltage - last.voltage).abs() > 0.15
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

                let mgr_ctx = manager_iface.signal_context();
                let mgr_inst = manager_iface.get().await;
                let _ = mgr_inst.on_battery_changed(mgr_ctx).await;
                let _ = mgr_inst.lid_is_closed_changed(mgr_ctx).await;

                println!("[EVENT] Cap: {}% | AC: {} | LidClosed: {}",
                    new_state.capacity,
                    if new_state.ac_present { "YES" } else { "NO" },
                    new_state.lid_closed
                );
            }
        }
    }
}