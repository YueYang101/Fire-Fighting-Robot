#!/usr/bin/env python3
import time, board, busio, numpy as np, cv2
import adafruit_mlx90640                                      # pip install adafruit-blinka adafruit-circuitpython-mlx90640
                                                              # pip install opencv-python
# ---------- hardware setup ----------
i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)      # or 1_000_000 once everything is stable
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_HZ # 8 FPS; try REFRESH_16_HZ after bus tuning

flat  = np.zeros(24 * 32, dtype=np.float32)                   # 1-D scratch buffer

# ---------- live loop ----------
while True:
    try:
        mlx.getFrame(flat)                                    # fills 'flat' in place
        frame = np.flipud(flat.reshape(24, 32))               # 2-D °C matrix, flipped so up = up

        # ---- convert to colour image ----
        img  = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX) \
                     .astype(np.uint8)                        # 0-255 8-bit
        img  = cv2.resize(img, (320, 240), interpolation=cv2.INTER_CUBIC)
        img  = cv2.applyColorMap(img, cv2.COLORMAP_TURBO)

        # ---- optional: overlay min / max temps ----
        min_val, max_val, *_ = cv2.minMaxLoc(frame)
        cv2.putText(img, f"min {min_val:4.1f}C", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.putText(img, f"max {max_val:4.1f}C", (5, 235),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        # ---- show window ----
        cv2.imshow("MLX90640 Heat-map – press q to quit", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except ValueError:                                        # sporadic CRC hiccup – just retry
        continue

cv2.destroyAllWindows()
