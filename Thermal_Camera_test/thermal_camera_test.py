import warnings
import time
import pathlib
import sys

import board
import busio
import numpy as np
import pandas as pd
import adafruit_mlx90640

# ─── USER SETTINGS ────────────────────────────────────────────────────────────
# If you changed /boot/firmware/config.txt to 100 kHz or 800 kHz, put that here.
I2C_FREQUENCY_HZ   = 400_000           # ignored on generic-linux but kept for clarity
EXCEL_FILENAME     = "thermal_frames.xlsx"
BACKOFF_BASE_S     = 0.1               # first retry waits 0.1 s, second 0.2 s …
BACKOFF_MAX_S      = 2.0               # … but never longer than 2 s
REFRESH_RATE       = adafruit_mlx90640.RefreshRate.REFRESH_0_5_HZ  # 0 .5 Hz = 2 s
# ───────────────────────────────────────────────────────────────────────────────

# Suppress the benign warning about I2C frequency
warnings.filterwarnings(
    "ignore",
    category=RuntimeWarning,
    message="I2C frequency is not settable in python, ignoring!"
)

# Lookup table: register value → frames-per-second
_FPS_LUT = {
    0x00: 0.5,
    0x01: 1,
    0x02: 2,
    0x03: 4,
    0x04: 8,
    0x05: 16,
    0x06: 32,
    0x07: 64,
}
FRAME_PERIOD_S = 1 / _FPS_LUT[REFRESH_RATE]

# ─── INITIALISE I²C AND SENSOR ────────────────────────────────────────────────
try:
    i2c = busio.I2C(board.SCL, board.SDA, frequency=I2C_FREQUENCY_HZ)
except TypeError:                       # older Blinka versions
    i2c = busio.I2C(board.SCL, board.SDA)

mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = REFRESH_RATE

# ─── PREP OUTPUT FILE ─────────────────────────────────────────────────────────
xlsx_path = pathlib.Path(__file__).with_name(EXCEL_FILENAME)
flat      = np.zeros(24 * 32, dtype=np.float32)
sheet_idx = 0

print(f"Logging one worksheet every {FRAME_PERIOD_S:.1f} s → {xlsx_path}")

# ─── MAIN LOOP ────────────────────────────────────────────────────────────────
retries = 0
try:
    while True:
        try:
            # Acquire a full 32 × 24 frame (blocks ≈ FRAME_PERIOD_S)
            mlx.getFrame(flat)
            frame   = flat.reshape((24, 32))
            retries = 0

            # Write to a new worksheet
            df      = pd.DataFrame(frame)
            stamp   = time.strftime("%Y-%m-%d_%Hh%Mm%Ss")

            with pd.ExcelWriter(
                    xlsx_path,
                    engine="openpyxl",
                    mode="a" if xlsx_path.exists() else "w",
                    if_sheet_exists="new") as w:
                df.to_excel(w, sheet_name=stamp,
                            index=False, header=False)

            sheet_idx += 1
            print(f"saved frame {sheet_idx:5d}  @ {stamp}")

        # CRC failure: back-off and retry
        except ValueError:
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"CRC error – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

        # Some I²C hiccups raise OSError; treat them the same way
        except OSError as e:
            wait = min(BACKOFF_BASE_S * (retries + 1), BACKOFF_MAX_S)
            retries += 1
            print(f"I²C error ({e}) – retrying in {wait:.1f} s (attempt {retries})",
                  file=sys.stderr, flush=True)
            time.sleep(wait)

except KeyboardInterrupt:
    print("\nStopped cleanly – Excel file is up-to-date.")