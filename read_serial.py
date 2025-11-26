import paramiko
import sys

# ---------------------------------------------------
# CONFIGURATION — EDIT THESE VALUES
# ---------------------------------------------------
HOST = "de-pi-228.local"
USER = "pi"
SSH_KEY_PATH = "/home/bachmak/.ssh/de-pi-228"  # ← your SSH private key
PORT = "/dev/ttyACM0"                       # Teensy serial port on the Pi
BAUD = 9600
# ---------------------------------------------------

# Python code injected into the Pi session
REMOTE_PYTHON = f"""
import serial, time, sys

PORT = "{PORT}"
BAUD = {BAUD}

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {{PORT}} at {{BAUD}} baud", flush=True)
except Exception as e:
    print("ERROR opening serial port:", e, flush=True)
    sys.exit(1)

while True:
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(line, flush=True)
    except KeyboardInterrupt:
        break
    except Exception as e:
        print("ERROR:", e, flush=True)
        time.sleep(0.1)
"""

def main():
    print(f"Connecting to {HOST} using private key {SSH_KEY_PATH}...")

    # Load SSH private key
    key = paramiko.Ed25519Key.from_private_key_file(SSH_KEY_PATH)

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    # Connect using the specified key
    ssh.connect(HOST, username=USER, pkey=key)

    # Run the serial reader on the Pi
    stdin, stdout, stderr = ssh.exec_command(
        f'python3 -u - << "EOF"\n{REMOTE_PYTHON}\nEOF'
    )

    print("Streaming Teensy output:\n")

    try:
        for line in stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ssh.close()


if __name__ == "__main__":
    main()
