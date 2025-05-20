#!/usr/bin/env python3
"""
thermal_to_excel.py  –  capture MLX90640 frames and save each one as a
                        worksheet inside thermal_frames.xlsx
"""
import time, os, pathlib, board, busio, numpy as np, pandas as pd
import adafruit_mlx90640

# ---------- ONE-TIME SENSOR SET-UP ----------
i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)           # 0.8 MHz bus
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ      # sensor delivers 1 FPS

# ---------- DESTINATION FILE ----------
xlsx_path = pathlib.Path(__file__).with_name("thermal_frames.xlsx")

# ---------- MAIN LOOP ----------
flat = np.zeros(24 * 32, dtype=np.float32)
print("Capturing one frame every 3 s →", xlsx_path)
try:
    while True:
        try:
            # 1) grab a frame
            mlx.getFrame(flat)
            frame = flat.reshape((24, 32))                         # 2-D °C matrix

            # 2) convert to DataFrame (32 columns, 24 rows)
            df = pd.DataFrame(frame)

            # 3) append to the Excel file (new sheet per frame)
            timestamp = time.strftime("%Y-%m-d_%Hh%Mm%Ss")
            with pd.ExcelWriter(
                    xlsx_path,
                    engine="openpyxl",
                    mode="a" if xlsx_path.exists() else "w",
                    if_sheet_exists="new") as writer:
                df.to_excel(writer, sheet_name=timestamp,
                             index=False, header=False)

            print(f"saved frame @ {timestamp}")
        except ValueError:                                         # CRC hiccup – ignore
            print("CRC error, skipping this frame")

        # 4) throttle to one capture every ~3 s (1 s capture + 2 s wait)
        time.sleep(2)

except KeyboardInterrupt:
    print("\nStopped by user – Excel file is up-to-date.")
