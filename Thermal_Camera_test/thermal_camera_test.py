#!/usr/bin/env python3
import time, pathlib, board, busio, numpy as np, pandas as pd
import adafruit_mlx90640

i2c = busio.I2C(board.SCL, board.SDA)               # kernel already at 400 kHz
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_0_5_HZ   # 2 s / frame

xlsx_path = pathlib.Path(__file__).with_name("thermal_frames.xlsx")
flat = np.zeros(24*32, dtype=np.float32)
sheet_idx = 0

print("Logging one worksheet every 2 s →", xlsx_path)
try:
    while True:
        try:
            mlx.getFrame(flat)                      # blocks ≈2 s
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

            sheet_idx += 1
            print(f"saved frame {sheet_idx:5d}  @ {stamp}")

        except ValueError:
            # extremely rare at 0.5 Hz; just retry next cycle
            print("CRC error – retrying")
            continue

except KeyboardInterrupt:
    print("\nStopped cleanly – Excel file is up-to-date.")
