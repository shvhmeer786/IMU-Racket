import time
import board
import busio
import adafruit_bno055

# ── UART connection to BNO055 ──────────────────────────────────────
uart = busio.UART(tx=board.TX, rx=board.RX, baudrate=115200)

time.sleep(1)                # let sensor boot

try:
    sensor = adafruit_bno055.BNO055_UART(uart)
    print("BNO055 connected. Streaming data…")
except Exception as e:
    print("Sensor init failed:", e)
    sensor = None

def fmt(vec):
    """Return 'x.xx, y.yy, z.zz'  or zeros if None."""
    return "0.00, 0.00, 0.00" if vec is None else f"{vec[0]:.2f}, {vec[1]:.2f}, {vec[2]:.2f}"

while True:
    if sensor is None:
        print("Sensor not initialized")
        time.sleep(0.5)
        continue

    # ── Orientation (Euler) ────────────────────────────────────────
    euler = sensor.euler                # (heading, roll, pitch)
    if euler:
        heading, roll, pitch = euler
        print(f"Orientation: {heading:.2f}, {roll:.2f}, {pitch:.2f}")
    else:
        print("Orientation: 0.00, 0.00, 0.00")

    # ── Raw accelerometer (includes gravity) ───────────────────────
    accel = sensor.acceleration         # (ax, ay, az)  m/s²
    print(f"Accel: {fmt(accel)}")

    # ── Linear acceleration (gravity-compensated) ──────────────────
    lin_accel = sensor.linear_acceleration
    print(f"LinAccel: {fmt(lin_accel)}")

    # ── Temperature ─────────────────────────────────────────────────
    temp = sensor.temperature
    print(f"Temp: {temp:.2f}" if temp is not None else "Temp: 0.00")

    # add other lines the same way if/when you need them:
    # gyro  = sensor.gyro
    # mag   = sensor.magnetic
    # quat  = sensor.quaternion

    time.sleep(0.01)        # 100 Hz output; reduce if you need faster
