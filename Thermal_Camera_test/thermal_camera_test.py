#!/usr/bin/env python3
"""
thermal_to_excel.py – save one MLX90640 frame every 3 seconds
into thermal_frames.xlsx (one worksheet per frame)
"""
import time, pathlib, board, busio, numpy as np, pandas as pd
import adafruit_mlx90640

# ---------- sensor ----------
i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ   # MUST read ≥1 Hz

# ---------- file ----------
xlsx_path = pathlib.Path(__file__).with_name("thermal_frames.xlsx")

flat   = np.zeros(24*32, dtype=np.float32)
frames = 0                             # counts reads, we save every 3rd

print("Logging one worksheet every 3 s →", xlsx_path)
try:
    while True:
        try:
            mlx.getFrame(flat)                         # 1 frame every second
            frames += 1
            if frames % 3:                             # skip 1st & 2nd, save 3rd
                continue

            frame = flat.reshape((24, 32))
            df    = pd.DataFrame(frame)

            stamp = time.strftime("%Y-%m-%d_%Hh%Mm%Ss")
            with pd.ExcelWriter(
                    xlsx_path,
                    engine="openpyxl",
                    mode="a" if xlsx_path.exists() else "w",
                    if_sheet_exists="new") as w:
                df.to_excel(w, sheet_name=stamp,
                            index=False, header=False)
            print(f"saved frame  #{frames//3:5d}  @ {stamp}")

        except ValueError:
            print("CRC error – retrying next second")
            continue                                   # stay on schedule

except KeyboardInterrupt:
    print("\nStopped.  Excel file is up-to-date.")
