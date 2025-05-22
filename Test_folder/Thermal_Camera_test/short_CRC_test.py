import time, board, busio, numpy as np, adafruit_mlx90640
i2c = busio.I2C(board.SCL, board.SDA)                 # no frequency arg
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_1_HZ
buf = np.zeros(24*32, dtype=np.float32)

good = bad = 0
t0 = time.time()
while time.time() - t0 < 30:                          # 30 s window
    try:
        mlx.getFrame(buf)
        good += 1
    except ValueError:
        bad += 1
print(f"good {good}, CRC {bad}")
