# Argon ONE UP UPS Daemon

An efficient Power Management Daemon for the **Argon ONE UP** (Laptop model) on Raspberry Pi, written in Rust.

This daemon replaces the original Python scripts and provides native integration into the system D-Bus, allowing battery status to be displayed directly in desktop environments like **KDE Plasma** or **GNOME**.

## Features

* **Native D-Bus Integration:** Simulates a UPower-compatible device.
* **Laptop Model Support:** Detects lid status via GPIO 27.
* **Stable AC Detection:** Advanced voltage-based logic to prevent "toggling" artifacts.
* **Power Button Support:** Graceful shutdown on long-press.
* **Optimized Performance:** Uses hardware interrupts for GPIOs and efficient I2C polling.

## Prerequisites

Before installing, ensure that I2C is enabled on your Raspberry Pi:

1. Run `sudo raspi-config`.
2. Navigate to **Interface Options** -> **I2C** and select **Yes**.
3. Reboot your Pi.

## Installation

### 1. Dependencies

Ensure Rust and Cargo are installed:

```bash
curl --proto '=https' --tlsv1.2 -sSf [https://sh.rustup.rs](https://sh.rustup.rs) | sh
```

### 2. Configure D-Bus Policy

To allow the daemon to reserve the `org.freedesktop.UPower` name on the system bus, copy the policy file to the correct directory:

```bash
sudo cp config/org.freedesktop.UPower.BatteryArgon.conf /etc/dbus-1/system.d/
sudo systemctl reload dbus
```

### 3. Build and Install

Build the optimized release binary and move it to your system path:

```bash
cargo build --release
sudo cp target/release/argon-one-up-daemon /usr/local/bin/
```

### 4. Setup Systemd Service

Install and start the background service:

```bash
sudo cp config/argon-one-up-daemon.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now argon-one-up-daemon
```

## License

This project is licensed under the **GPL-3.0 License**. See the `LICENSE` file for the full text.