import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import math
import threading
from collections import deque

# === KONFIGURACJA SIECIOWA ===
# Adres 0.0.0.0 - komputer nasłuchuje na wszystkich swoich interfejsach
UDP_IP = "0.0.0.0"
UDP_PORT = 4444

MAX_POINTS = 10   # Długość śladu ruchu
MAX_SCAN_POINTS = 10 # Długość śladu dla skanera (tło)
MAX_SHOTS = 1     # Ilość zapamiętanych strzałów

# === BUFORY DANYCH ===
# Ślady ruchu (niebieskie kropki)
angles_h = deque(maxlen=MAX_POINTS)
distances_h = deque(maxlen=MAX_POINTS)
angles_v = deque(maxlen=MAX_POINTS)
distances_v = deque(maxlen=MAX_POINTS)

# Ślady skanow
scan_h_ang = deque(maxlen=MAX_SCAN_POINTS)
scan_h_dist = deque(maxlen=MAX_SCAN_POINTS)
scan_v_ang = deque(maxlen=MAX_SCAN_POINTS)
scan_v_dist = deque(maxlen=MAX_SCAN_POINTS)

# Ślady strzałów (czerwone gwiazdki)
shots_h_ang = deque(maxlen=MAX_SHOTS)
shots_h_dist = deque(maxlen=MAX_SHOTS)
shots_v_ang = deque(maxlen=MAX_SHOTS)
shots_v_dist = deque(maxlen=MAX_SHOTS)

# === PAMIĘĆ STANU ===
last_pos_h = (0.0, 0.0) 
last_pos_v = (0.0, 0.0)

running = True

def read_udp_thread():
    # Wątek odczytu: nasłuchuje pakietów UDP
    global last_pos_h, last_pos_v
    
    # Utworzenie gniazda UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        # Początek nasłuchiwania
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(1.0) # Timeout na wypadek zakończenia programu
        print(f"Nasłuchiwanie UDP na porcie {UDP_PORT}...")
    except Exception as e:
        print(f"Błąd uruchamiania gniazda sieciowego: {e}")
        return

    while running:
        try:
            # Odbiór danych (bufor 1024 bajty)
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()

            lines = message.split('\n')

            for line in lines:
                line = line.strip()
                if not line:
                    continue
                
                # --- OBSŁUGA STRZAŁU ---
                if line == 'S':
                    print(f"!!! STRZAŁ !!! [od {addr[0]}]") 
                    shots_h_ang.append(last_pos_h[0])
                    shots_h_dist.append(last_pos_h[1])
                    shots_v_ang.append(last_pos_v[0])
                    shots_v_dist.append(last_pos_v[1])
                    continue 

                # --- OBSŁUGA RUCHU (format: H,kąt,dystans) ---
                parts = line.split(',')
                if len(parts) != 3:
                    continue
                
                plane = parts[0]
                try:
                    # Konwersja formatu danych z Pico na format wykresu
                    angle_raw = float(parts[1])
                    distance = float(parts[2])
                    
                    # Logika przeliczania kąta
                    angle = math.radians((angle_raw - 90)*-1) 
                except ValueError:
                    continue

                if plane == 'H':      # Ruch Poziom
                    angles_h.append(angle)
                    distances_h.append(distance)
                    last_pos_h = (angle, distance)
                elif plane == 'V':    # Ruch Pion
                    angles_v.append(angle)
                    distances_v.append(distance)
                    last_pos_v = (angle, distance)
                elif plane == 'h':    # Skaner Poziom
                    print(f"Skaner H: kąt={angle_raw}, dystans={distance}")
                    scan_h_ang.append(angle)
                    scan_h_dist.append(distance)
                elif plane == 'v':    # Skaner Pion
                    print(f"Skaner V: kąt={angle_raw}, dystans={distance}")
                    scan_v_ang.append(angle)
                    scan_v_dist.append(distance)

        except socket.timeout:
            continue
        except Exception as e:
            print(f"Błąd odczytu danych: {e}")
            break
    
    sock.close()

def update(frame):
    # Aktualizacja wykresów ruchu i strzałów
    
    scan_line_h.set_xdata(scan_h_ang)
    scan_line_h.set_ydata(scan_h_dist)
    
    scan_line_v.set_xdata(scan_v_ang)
    scan_line_v.set_ydata(scan_v_dist)

    line_h.set_xdata(angles_h)
    line_h.set_ydata(distances_h)
    
    line_v.set_xdata(angles_v)
    line_v.set_ydata(distances_v)

    shot_plot_h.set_xdata(shots_h_ang)
    shot_plot_h.set_ydata(shots_h_dist)
    
    shot_plot_v.set_xdata(shots_v_ang)
    shot_plot_v.set_ydata(shots_v_dist)
    
    return line_h, line_v, shot_plot_h, shot_plot_v, scan_line_h, scan_line_v

# === INICJALIZACJA WYKRESÓW ===
fig = plt.figure(figsize=(10, 5))

# --- RADAR POZIOMY ---
ax1 = fig.add_subplot(1, 2, 1, projection='polar')
ax1.set_title("Radar poziomy (Azymut)")
ax1.set_theta_zero_location("N")
ax1.set_theta_direction(-1)
ax1.grid(True)
ax1.set_ylim(0, 200) 
scan_line_h, = ax1.plot([], [], 'bo-', color='#705749', alpha=0.7, zorder=1, markersize=2, linewidth=1, label='Skaner')
line_h, = ax1.plot([], [], 'bo-', markersize=3, zorder=2, linewidth=1, label='Cel')
shot_plot_h, = ax1.plot([], [], 'r*', markersize=15, zorder=3, linestyle='None', label='Strzał')
ax1.legend(loc='lower right', bbox_to_anchor=(1.3, -0.1))

# --- RADAR PIONOWY ---
ax2 = fig.add_subplot(1, 2, 2, projection='polar')
ax2.set_title("Radar pionowy (Elewacja)")
ax2.set_theta_zero_location("E")
ax2.set_theta_direction(-1)
ax2.grid(True)
ax2.set_ylim(0, 200)
scan_line_v, = ax2.plot([], [], 'bo-', color='#705749', alpha=0.7, zorder=1, markersize=2, linewidth=1)
line_v, = ax2.plot([], [], 'ro-', markersize=3, zorder=2, linewidth=1, color='blue')
shot_plot_v, = ax2.plot([], [], 'r*', markersize=15, zorder=3, linestyle='None')

# Uruchomienie wątku UDP
thread = threading.Thread(target=read_udp_thread, daemon=True)
thread.start()

# Animacja
ani = animation.FuncAnimation(fig, update, interval=50, blit=True)

plt.tight_layout()
try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    running = False
    thread.join(timeout=1.0)