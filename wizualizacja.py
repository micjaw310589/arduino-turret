import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import math

# === KONFIGURACJA PORTU BLUETOOTH ===
PORT = "COM3" # Zmień na swój port Bluetooth
BAUDRATE = 9600
TIMEOUT = 1

# Bufory danych
angles_h = [] # poziome
distances_h = []
angles_v = [] # pionowe
distances_v = []
def read_serial_data():

    #Odczytuje jedną linię z portu szeregowego.
    #Format: H,kat,odległość lub V,kat,odległość
    try:
        line = ser.readline().decode('utf-8').strip()
        if not line:
            return None, None, None
        parts = line.split(',')
        if len(parts) != 3:
            return None, None, None
        plane = parts[0].upper() # 'H' lub 'V'
        angle = float(parts[1])
        distance = float(parts[2])
        return plane, angle, distance
    except (ValueError, UnicodeDecodeError):
        return None, None, None

def update(frame):

    #Aktualizacja wykresów.
    plane, angle, distance = read_serial_data()
    if plane and angle is not None and distance is not None:
        if plane == 'H':
            angles_h.append(angle)
            distances_h.append(distance)
            if len(angles_h) > 360:
                angles_h.pop(0)
                distances_h.pop(0)
        elif plane == 'V':
            angles_v.append(angle)
            distances_v.append(distance)
            if len(angles_v) > 360:
                angles_v.pop(0)
                distances_v.pop(0)

    # Wykres poziomy
    ax1.clear()
    ax1.set_title("Radar poziomy")
    ax1.set_theta_zero_location("N") # 0° na górze
    ax1.set_theta_direction(-1) # zgodnie z ruchem wskazówek zegara
    ax1.grid(True)
    ax1.plot([math.radians(a) for a in angles_h], distances_h, marker='o', color='b')

    # Wykres pionowy (obrócony o 90°)
    ax2.clear()
    ax2.set_title("Radar pionowy")
    ax2.set_theta_zero_location("E") # 0° w prawo (90° przesunięcia)
    ax2.set_theta_direction(-1)
    ax2.grid(True)
    ax2.plot([math.radians(a) for a in angles_v], distances_v, marker='o', color='r')

# === Inicjalizacja połączenia ===
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
    print(f"Połączono z {PORT} przy {BAUDRATE} bps")
except serial.SerialException as e:
    print(f"Błąd połączenia z portem szeregowym: {e}")
    sys.exit(1)

# === Przygotowanie wykresów ===
fig = plt.figure(figsize=(10, 5))
ax1 = fig.add_subplot(1, 2, 1, projection='polar')
ax2 = fig.add_subplot(1, 2, 2, projection='polar')

ani = animation.FuncAnimation(fig, update, interval=200)
plt.tight_layout()
plt.show()

# Zamknięcie portu po zakończeniu
ser.close()