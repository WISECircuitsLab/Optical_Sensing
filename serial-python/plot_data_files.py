#!/usr/bin/env python3
"""
plot_data_files.py

- Loads CSV files produced by serial_monitor.py in ./data/
- For each CSV: parses timestamps and A0 voltages, builds measured-time axis,
  interpolates to uniform grid if needed, plots:
    (1) voltage and running average vs measured time
    (2) single-sided FFT power spectrum in dB vs log frequency
- Saves plot as <csvname>_plot.png next to each CSV.
- Run: python plot_data_files.py
"""

import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from scipy import interpolate
from numpy.fft import fft

DATA_DIR = 'data'
CSV_GLOB = os.path.join(DATA_DIR, 'datafile_*.csv')
replot = False  # If True, replot all files. If False, only plot files without existing plot PNGs.

def load_file(path):
    df = pd.read_csv(path, dtype=str)
    # normalize columns - they were written as strings, convert where possible
    # Attempt to parse datetime_iso
    if 'datetime_iso' in df.columns:
        df['datetime_iso_parsed'] = pd.to_datetime(df['datetime_iso'], errors='coerce')
    else:
        df['datetime_iso_parsed'] = pd.to_datetime(df['local_epoch_s'].astype(float), unit='s', errors='coerce')

    # parse numeric A0 columns
    if 'A0_voltage_V' in df.columns:
        df['A0_voltage'] = pd.to_numeric(df['A0_voltage_V'], errors='coerce')
    else:
        df['A0_voltage'] = np.nan

    if 'A0_avg_V' in df.columns:
        df['A0_avg'] = pd.to_numeric(df['A0_avg_V'], errors='coerce')
    else:
        df['A0_avg'] = np.nan

    return df

def make_uniform_samples(t_seconds, y, method='pchip'):
    # If t_seconds irregular, resample to uniform with median dt
    dt = np.median(np.diff(t_seconds))
    if dt <= 0 or np.isnan(dt) or len(t_seconds) < 2:
        # fallback to simple linear spacing
        dt = 0.1
    t_uniform = np.arange(t_seconds[0], t_seconds[-1] + dt/2, dt)
    # use pchip-like behavior via cubic or linear depending on availability
    try:
        kind = 'cubic' if method == 'pchip' else 'linear'
        f = interpolate.interp1d(t_seconds, y, kind=kind, fill_value="extrapolate")
        y_uniform = f(t_uniform)
    except Exception:
        # fallback linear interp
        f = interpolate.interp1d(t_seconds, y, kind='linear', fill_value="extrapolate")
        y_uniform = f(t_uniform)
    return t_uniform, y_uniform, dt

def single_sided_fft(y, fs):
    N = len(y)
    if N < 2:
        return np.array([0.0]), np.array([0.0])
    Y = fft(y)
    P2 = np.abs(Y / N)
    if N % 2 == 0:
        P1 = P2[:N//2 + 1]
        P1[1:-1] = 2 * P1[1:-1]
    else:
        P1 = P2[:(N+1)//2]
        P1[1:] = 2 * P1[1:]
    f = np.arange(len(P1)) * (fs / N)
    return f, P1

def plot_one(df, path):
    basename = os.path.splitext(os.path.basename(path))[0]
    # Select rows that have parsed datetime and A0 voltage
    df_use = df.dropna(subset=['datetime_iso_parsed', 'A0_voltage'])
    if df_use.empty:
        print(f"No usable A0 data in {path}, skipping.")
        return
    t_dt = df_use['datetime_iso_parsed'].values
    # convert to seconds relative to first sample
    t_seconds = (t_dt - t_dt[0]) / np.timedelta64(1, 's')
    t_seconds = t_seconds.astype(float)
    V = df_use['A0_voltage'].astype(float).values
    Vavg = df_use['A0_avg'].astype(float).values

    # Create uniform grid and compute FFT
    t_u, V_u, dt = make_uniform_samples(t_seconds, V, method='pchip')
    fs = 1.0 / dt
    f, P1 = single_sided_fft(V_u, fs)
    
    # Convert amplitude to power and then to dB
    # Power = amplitude^2, then dB = 10*log10(power) = 20*log10(amplitude)
    # Avoid log(0) by adding small epsilon
    epsilon = 1e-10
    power = P1 ** 2
    power_db = 10 * np.log10(power + epsilon)
    
    # Filter out DC component (f=0) and negative frequencies for log plot
    # Also filter out any invalid values
    valid_mask = (f > 0) & np.isfinite(power_db)
    f_plot = f[valid_mask]
    power_db_plot = power_db[valid_mask]
    
    # Build figure
    fig, axs = plt.subplots(2,1, figsize=(10,7), constrained_layout=True)
    axs[0].plot(t_seconds, V, marker='o', linestyle='-', label='A0 Voltage')
    if not np.all(np.isnan(Vavg)):
        axs[0].plot(t_seconds, Vavg, marker='s', linestyle='--', label='Running Avg')
    axs[0].set_xlabel('Time (s) relative')
    axs[0].set_ylabel('Voltage (V)')
    axs[0].set_title(f'{basename} - Voltage vs Time')
    axs[0].legend()
    axs[0].grid(True)

    # Plot power spectrum in dB vs log frequency
    axs[1].semilogx(f_plot, power_db_plot, linestyle='-')
    axs[1].set_xlabel('Frequency (Hz) - Log Scale')
    axs[1].set_ylabel('Power (dB)')
    axs[1].set_title('Single-sided FFT Power Spectrum')
    axs[1].grid(True, which='both')  # Show both major and minor grid lines for log scale
    if len(f_plot) > 0:
        axs[1].set_xlim(f_plot.min(), min(fs/2, f_plot.max()))

    outpng = os.path.join(os.path.dirname(path), f"{basename}_plot.png")
    fig.suptitle(basename)
    fig.savefig(outpng, dpi=150)
    plt.close(fig)
    print(f"Saved plot: {outpng}")

def main():
    files = sorted(glob.glob(CSV_GLOB))
    if not files:
        print("No files found in data/ matching datafile_*.csv")
        return
    
    if not replot:
        print(f"replot={replot}: Only plotting files without existing plots...")
    else:
        print(f"replot={replot}: Replotting all files...")
    
    plotted_count = 0
    skipped_count = 0
    
    for p in files:
        # Check if plot file already exists
        basename = os.path.splitext(os.path.basename(p))[0]
        plot_path = os.path.join(os.path.dirname(p), f"{basename}_plot.png")
        
        if not replot and os.path.exists(plot_path):
            print(f"Skipping {os.path.basename(p)} (plot already exists: {os.path.basename(plot_path)})")
            skipped_count += 1
            continue
        
        try:
            df = load_file(p)
            plot_one(df, p)
            plotted_count += 1
        except Exception as e:
            print("Error processing", p, e)
    
    print(f"\nSummary: {plotted_count} file(s) plotted, {skipped_count} file(s) skipped")

if __name__ == '__main__':
    main()
