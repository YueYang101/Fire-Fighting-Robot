import time, board, busio, numpy as np
import adafruit_mlx90640

i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ   # 4 FPS

flat = np.zeros(24 * 32, dtype=float)          # 1-D buffer

while True:
    try:
        mlx.getFrame(flat)                     # fill buffer in place
        temp_matrix = flat.reshape((24, 32))   # now 2-D
        print(temp_matrix)                     # or do any processing here
    except ValueError:                         # CRC hiccup, just retry
        continue
