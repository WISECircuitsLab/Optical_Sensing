#!/usr/bin/env python3
"""
serial_monitor.py

- Connects to a serial device (default port guesses provided).
- Selects LMP2, sets gain to maximum (7), toggles continuous 'C' mode ON.
- Logs incoming serial lines to a timestamped CSV in ./data/
- Attempts to parse A0 voltage lines and the CSV-status lines emitted by the Arduino.
- Run: python serial_monitor.py --port /dev/ttyUSB0
"""

import argparse
import os
import re
import sys
import csv
import time
from datetime import datetime
import serial

# ---------- User-config ----------
DEFAULT_PORTS = ['/dev/ttyUSB0', '/dev/ttyACM0', 'COM3', 'COM4']
BAUDRATE = 115200
SERIAL_TIMEOUT = 1.0  # seconds for reads
DATA_DIR = 'data'
AVERAGE_DISPLAY_INTERVAL = 30.0  # seconds - display average reading every N seconds
# ---------------------------------

# Regular expressions to parse known line formats
RE_A0 = re.compile(r'A0 Voltage:\s*([+-]?\d*\.?\d+)\s*V\s*,\s*Avg:\s*([+-]?\d*\.?\d+)\s*V', re.IGNORECASE)
RE_CSV = re.compile(r'^\s*(\d+)\s*,\s*([^,]+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*([^,]+)\s*,\s*(ON|OFF|True|False)\s*', re.IGNORECASE)
RE_STATUS_TIA = re.compile(r'^\s*TIA:\s*(\d+)', re.IGNORECASE)
RE_STATUS_GAIN = re.compile(r'^\s*Gain Level:\s*(\d+)', re.IGNORECASE)

def find_port(user_port=None):
    if user_port:
        return user_port
    for p in DEFAULT_PORTS:
        try:
            ser = serial.Serial(p, BAUDRATE, timeout=0.1)
            ser.close()
            return p
        except Exception:
            pass
    raise RuntimeError("No serial port specified and default ports not available. Provide --port argument.")

def open_serial(port):
    ser = serial.Serial(port, BAUDRATE, timeout=SERIAL_TIMEOUT)
    # flush input a bit
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser

def write_and_drain(ser, s):
    ser.write((s + '\n').encode('ascii'))
    ser.flush()

def send_and_wait_for_pattern(ser, cmd, pattern_re, attempts=4, wait_after=0.2, read_timeout=1.0):
    """Send cmd (single char or string), then read lines for pattern_re up to attempts times.
       Returns matched group or None."""
    for attempt in range(attempts):
        write_and_drain(ser, cmd)
        # read for up to read_timeout seconds
        deadline = time.time() + read_timeout
        while time.time() < deadline:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            # print("DBG RX:", line)
            m = pattern_re.search(line)
            if m:
                return m
            time.sleep(0.01)
        time.sleep(wait_after)
    return None

def ensure_tia_is(ser, target_tia):
    """
    Attempts to set the device to TIA index target_tia (0..3).
    Strategy: send 'n' repeatedly up to 8 times and check for "TIA: <num>" status lines.
    """
    # try to query current status by reading lines for a moment
    # fallback: cycle 'n' while checking for TIA
    max_cycles = 8
    for i in range(max_cycles):
        # ask for next TIA; device will print statuses after state changes
        write_and_drain(ser, 'n')
        deadline = time.time() + 0.8
        while time.time() < deadline:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            m = RE_STATUS_TIA.search(line)
            if m:
                cur = int(m.group(1))
                if cur == target_tia:
                    return True
                # if not target, continue cycling
            # keep reading other lines
        # small pause
        time.sleep(0.05)
    # final attempt: brute-force rotate to known position by sending 'n' until we see target
    for i in range(12):
        write_and_drain(ser, 'n')
        time.sleep(0.08)
        # read lines
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            m = RE_STATUS_TIA.search(line)
            if m:
                cur = int(m.group(1))
                if cur == target_tia:
                    return True
    return False

def ensure_gain_is_max(ser, target_gain=7):
    """
    Sends 'u' repeatedly until device reports Gain Level: target_gain (or up to attempts).
    """
    attempts = 12
    for i in range(attempts):
        # read any pre-existing lines
        while ser.in_waiting:
            _ = ser.readline()
        write_and_drain(ser, 'u')
        deadline = time.time() + 0.6
        while time.time() < deadline:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            m = RE_STATUS_GAIN.search(line)
            if m:
                cur = int(m.group(1))
                if cur >= target_gain:
                    return True
            # continue reading until deadline
    return False

def toggle_continuous_on(ser):
    # send 'C' to toggle. We assume off->on. To be safe, send once and let it be.
    write_and_drain(ser, 'C')
    # read responses for a short time
    deadline = time.time() + 0.6
    seen = False
    while time.time() < deadline:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        if 'Continuous' in line or 'CONTINUOUS' in line or 'stream' in line.lower():
            seen = True
        # if the device prints CSV header, it's fine
    return seen

def make_output_file():
    os.makedirs(DATA_DIR, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    fname = f"datafile_{ts}.csv"
    path = os.path.join(DATA_DIR, fname)
    return path

def monitor_loop(ser, outfile_path):
    # Open CSV writer and write header
    fieldnames = [
        'datetime_iso', 'local_epoch_s', 'device_ms', 'command', 'TIA', 'Gain_Level',
        'R_External', 'R_Internal', 'R_Total', 'LED_State',
        'A0_voltage_V', 'A0_avg_V', 'raw'
    ]
    with open(outfile_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        print(f'Logging to: {outfile_path}')
        
        # Periodic average tracking
        a0_readings = []  # List of (timestamp, voltage) tuples
        last_average_display = time.time()
        start_time = time.time()
        
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                now = datetime.now()
                current_time = time.time()
                row = dict.fromkeys(fieldnames, '')
                row['datetime_iso'] = now.isoformat(sep=' ')
                row['local_epoch_s'] = f"{now.timestamp():.6f}"
                row['raw'] = line

                # try parse CSV-style status line
                m_csv = RE_CSV.match(line)
                if m_csv:
                    # groups correspond to Arduino CSV: Timestamp(ms),Command,TIA,Gain_Level,R_External,R_Internal,R_Total,LED_State
                    row['device_ms'] = m_csv.group(1)
                    row['command'] = m_csv.group(2)
                    row['TIA'] = m_csv.group(3)
                    row['Gain_Level'] = m_csv.group(4)
                    row['R_External'] = m_csv.group(5)
                    row['R_Internal'] = m_csv.group(6)
                    row['R_Total'] = m_csv.group(7)
                    row['LED_State'] = m_csv.group(8)
                    writer.writerow(row)
                    csvfile.flush()
                    print(f"[{row['datetime_iso']}] CSV: cmd={row['command']} TIA={row['TIA']} Gain={row['Gain_Level']}")
                    continue

                # try parse A0 line
                m_a0 = RE_A0.search(line)
                if m_a0:
                    voltage = float(m_a0.group(1))
                    avg_voltage = float(m_a0.group(2))
                    row['A0_voltage_V'] = voltage
                    row['A0_avg_V'] = avg_voltage
                    writer.writerow(row)
                    csvfile.flush()
                    print(f"[{row['datetime_iso']}] A0={voltage:.4f} V  Avg={avg_voltage:.4f} V")
                    
                    # Track A0 readings for periodic average
                    a0_readings.append((current_time, voltage))
                    
                    # Check if it's time to display periodic average
                    elapsed_since_last_display = current_time - last_average_display
                    if elapsed_since_last_display >= AVERAGE_DISPLAY_INTERVAL:
                        # Calculate average over the interval
                        interval_start = last_average_display
                        readings_in_interval = [(t, v) for t, v in a0_readings if t >= interval_start]
                        
                        if readings_in_interval:
                            voltages_in_interval = [v for _, v in readings_in_interval]
                            interval_avg = sum(voltages_in_interval) / len(voltages_in_interval)
                            interval_min = min(voltages_in_interval)
                            interval_max = max(voltages_in_interval)
                            elapsed_total = current_time - start_time
                            
                            print(f"\n{'='*60}")
                            print(f"PERIODIC AVERAGE (last {AVERAGE_DISPLAY_INTERVAL:.1f}s):")
                            print(f"  Average: {interval_avg:.6f} V")
                            print(f"  Min:     {interval_min:.6f} V")
                            print(f"  Max:     {interval_max:.6f} V")
                            print(f"  Range:   {interval_max - interval_min:.6f} V")
                            print(f"  Samples: {len(readings_in_interval)}")
                            print(f"  Total elapsed: {elapsed_total:.1f} s")
                            print(f"{'='*60}\n")
                        
                        last_average_display = current_time
                        # Keep only recent readings (last 2 intervals worth) to prevent memory growth
                        cutoff_time = current_time - (2 * AVERAGE_DISPLAY_INTERVAL)
                        a0_readings = [(t, v) for t, v in a0_readings if t >= cutoff_time]
                    
                    continue

                # otherwise write raw line
                writer.writerow(row)
                csvfile.flush()
                print(f"[{row['datetime_iso']}] RAW: {line}")
        except KeyboardInterrupt:
            print("Interrupted by user, closing.")
        except Exception as e:
            print("Error in monitor loop:", e)
        finally:
            try:
                ser.close()
            except Exception:
                pass

def main():
    parser = argparse.ArgumentParser(description='Serial monitor & logger for LMP91000 device')
    parser.add_argument('--port', '-p', type=str, default=None, help='Serial port (e.g. /dev/ttyUSB0 or COM3)')
    parser.add_argument('--set-tia', type=int, default=1, help='Target TIA index (0..3). Default: 1 (LMP2)')
    parser.add_argument('--target-gain', type=int, default=7, help='Target internal gain level (0..7). Default: 7 (max)')
    args = parser.parse_args()

    port = find_port(args.port)
    print(f"Opening serial port: {port} @ {BAUDRATE} baud")
    ser = open_serial(port)
    time.sleep(0.2)

    # Give device a moment to print initial lines
    time.sleep(0.5)
    # drain a bit of startup output
    while ser.in_waiting:
        _ = ser.readline()

    # Ensure TIA
    print(f"Setting TIA to {args.set_tia} (LMP{args.set_tia+1}) ...")
    ok_tia = ensure_tia_is(ser, args.set_tia)
    print("TIA set:", ok_tia)

    # Ensure gain
    print(f"Setting internal gain to {args.target_gain} (max) ...")
    ok_gain = ensure_gain_is_max(ser, args.target_gain)
    print("Gain set:", ok_gain)

    # Toggle continuous mode ON
    print("Toggling continuous streaming ON (sending 'C') ...")
    ok_cont = toggle_continuous_on(ser)
    print("Device acknowledged continuous toggle:", ok_cont)

    # Create output file and start monitoring
    outpath = make_output_file()
    monitor_loop(ser, outpath)

if __name__ == '__main__':
    main()
