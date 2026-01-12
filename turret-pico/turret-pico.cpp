#include <stdio.h>
#include <random>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/structs/rosc.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "btstack.h"

// Bluetooth SPP
static uint16_t rfcomm_channel_id = 0;
static uint8_t spp_service_buffer[150];
static bool bt_connected = false;

// Definicje Pinów
constexpr int ECHO_PIN = 6;
constexpr int TRIG_PIN = 7;
constexpr int LED_PIN = 25;
constexpr int LASER_PIN = 8;
constexpr int SERVO_SCANNER_HOR_PIN = 2;
constexpr int SERVO_SCANNER_VER_PIN = 3;
constexpr int SERVO_GUN_HOR_PIN = 4;
constexpr int SERVO_GUN_VER_PIN = 5;
constexpr int BUZZER_PIN = 9;
constexpr int BUTTON_PIN = 10;

// Zakres odległości do wykrywania celu (cm)
constexpr float MAX_SCANNER_DIST_CM = 200.0f;
constexpr float MIN_SCANNER_DIST_CM = 20.0f;

// Zakresy ruchu serw
constexpr int SCANNER_MIN_HOR_ANGLE = 30;
constexpr int SCANNER_MAX_HOR_ANGLE = 150;
constexpr int SCANNER_MIN_VER_ANGLE = 80;
constexpr int SCANNER_MAX_VER_ANGLE = 120;
constexpr int GUN_MIN_HOR_ANGLE = 30;
constexpr int GUN_MAX_HOR_ANGLE = 150;
constexpr int GUN_MIN_VER_ANGLE = 80;
constexpr int GUN_MAX_VER_ANGLE = 120;

// Domyślne nastawy serw
constexpr int DEFAULT_SCANNER_HOR_ANGLE = 90;
constexpr int DEFAULT_SCANNER_VER_ANGLE = 90;
constexpr int DEFAULT_GUN_HOR_ANGLE = 90;
constexpr int DEFAULT_GUN_VER_ANGLE = 90;

// Mapa odległości [ver][hor] dla kątów nastawu serw skanera
constexpr int ROW_NUM = 181;
constexpr int COL_NUM = 181;
float distanceMap[ROW_NUM][COL_NUM] = {999999.0f};


// Deklaracje funkcji i klas
class Servo {
    uint pin;
    uint slice_num;
    uint channel;

public:
    void attach(uint gpio_pin);
    void write(int angle);
};

struct ServoPosition {
    int VER;
    int HOR;

    bool isWithinAngleRange() {
        return (VER >= GUN_MIN_VER_ANGLE && VER <= GUN_MAX_VER_ANGLE &&
                HOR >= GUN_MIN_HOR_ANGLE && HOR <= GUN_MAX_HOR_ANGLE);
    }
};

struct AimPosition : ServoPosition {
    float DIST_CM;
};

// Pamięć odczytów -- bufor cykliczny
constexpr int SCAN_BUFFER_SIZE = 10;
AimPosition scanBuffer[SCAN_BUFFER_SIZE] = {-1, -1, 999999.0f};

void seed_random_from_rosc();
uint64_t get_pulse_width(uint pin);

ServoPosition parallax_correction(const AimPosition &target_pos);

void set_position(Servo &servo_ver, Servo &servo_hor, const ServoPosition &position);
float read_distance_cm();
void led_on_if_in_range(float distance_cm);
void write_distance_to_buffer(const ServoPosition &scanner_servo, float distance_cm);
void write_distance_to_map(const ServoPosition &scanner_servo, float distance_cm);
AimPosition get_position_to_target();
void shoot();
<<<<<<< Updated upstream
void random_deg_pos(ServoPosition &scanner_servo);
void random_scan_next_step(Servo &ver_servo, Servo &hor_servo, ServoPosition &scanner_servo);
void linear_scan_next_step(Servo &ver_servo, Servo &hor_servo, ServoPosition &scanner_servo);
=======
void random_deg_pos(int &ver_pos_deg, int &hor_pos_deg);
void random_scan_next_step(Servo &ver_servo, Servo &hor_servo, int &ver_pos_deg, int &hor_pos_deg);
void linear_scan_next_step(Servo &ver_servo, Servo &hor_servo, int &ver_pos_deg, int &hor_pos_deg, 
                      bool &horizontalDirection, bool &verticalDirection);
void bt_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
void bt_send_measurement(char axis, int angle, float distance);
>>>>>>> Stashed changes

// Globalne obiekty serw
Servo servo_scanner_hor;
Servo servo_scanner_ver;
Servo servo_gun_hor;
Servo servo_gun_ver;

// Pozycje serw
// constexpr ServoPosition scanner_start_pos = {SCANNER_MIN_VER_ANGLE, SCANNER_MAX_HOR_ANGLE};
ServoPosition scanner_servo_pos = {SCANNER_MIN_VER_ANGLE, SCANNER_MAX_HOR_ANGLE};
ServoPosition gun_servo_pos = {DEFAULT_GUN_VER_ANGLE, DEFAULT_GUN_HOR_ANGLE};


void setup() {
    // Inicjalizacja Serial (USB)
    stdio_init_all();

    // Inicjalizacja Bluetooth
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi/BT module\n");
        return;
    }

    // Inicjalizacja BTstack dla Bluetooth Classic
    l2cap_init();
    rfcomm_init();
    rfcomm_register_service(bt_packet_handler, RFCOMM_SERVER_CHANNEL_1, 0xffff);

    // Konfiguracja SPP (Serial Port Profile)
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL_1, "Turret SPP");
    sdp_register_service(spp_service_buffer);

    // Konfiguracja Bluetooth jako widoczne urządzenie
    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("Pico-Turret");

    // Start HCI
    hci_power_control(HCI_POWER_ON);

    printf("Bluetooth initialized. Device name: Pico-Turret\n");

    // Inicjalizacja rng
    seed_random_from_rosc();

    // Konfiguracja pinów
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_init(LASER_PIN);
    gpio_set_dir(LASER_PIN, GPIO_OUT);
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    // Podłączenie serw
    servo_scanner_hor.attach(SERVO_SCANNER_HOR_PIN);
    servo_scanner_ver.attach(SERVO_SCANNER_VER_PIN);
    servo_gun_hor.attach(SERVO_GUN_HOR_PIN);
    servo_gun_ver.attach(SERVO_GUN_VER_PIN);

    // Ustawienie pozycji początkowych
    servo_scanner_hor.write(DEFAULT_SCANNER_HOR_ANGLE);
    sleep_ms(300);
    servo_scanner_ver.write(DEFAULT_SCANNER_VER_ANGLE);
    sleep_ms(300);
    servo_gun_hor.write(DEFAULT_GUN_HOR_ANGLE);
    sleep_ms(300);
    servo_gun_ver.write(DEFAULT_GUN_VER_ANGLE);
    sleep_ms(300);

    set_position(servo_scanner_ver, servo_scanner_hor, scanner_servo_pos);
}


int main() {

    setup();

    while (true) {
        // Przetwarzanie zdarzeń Bluetooth
        btstack_run_loop_execute_once();

        float distance_cm = read_distance_cm();
        printf("Zmierzona odległość: %.2f cm\n", distance_cm);

        write_distance_to_buffer(scanner_servo_pos, distance_cm);
        // write_distance_to_map(scanner_servo_pos, distance_cm);

        // Wysyłanie pomiarów przez Bluetooth
        if (bt_connected) {
            // Wysyłanie pozycji poziomej (H)
            bt_send_measurement('H', scanner_servo_pos.HOR, distance_cm);
            // Wysyłanie pozycji pionowej (V)
            bt_send_measurement('V', scanner_servo_pos.VER, distance_cm);
        }

        AimPosition target_pos = get_position_to_target();

        if (target_pos.HOR != -1 && target_pos.VER != -1) {
            // Wycelowanie
            ServoPosition target_for_gun = parallax_correction(target_pos);
            set_position(servo_gun_ver, servo_gun_hor, target_for_gun);

            // Wciśnięcie przycisku do strzału
            if (!gpio_get(BUTTON_PIN)) {
                printf("Strzał w cel na odległości: %.2f cm (ver: %d, hor: %d)\n", target_pos.DIST_CM, target_pos.VER, target_pos.HOR);
                shoot();
            }
            sleep_ms(10);   // Ochrona przed debouncingiem
        }else {
            set_position(servo_gun_ver, servo_gun_hor, gun_servo_pos);
            printf("Brak celu w zasięgu.\n");
        }

        // random_scan_next_step(servo_scanner_ver, servo_scanner_hor, scanner_servo_pos);
        linear_scan_next_step(servo_scanner_ver, servo_scanner_hor, scanner_servo_pos);
    }
    
    return 0;
}



void Servo::attach(uint gpio_pin) {
    pin = gpio_pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(pin);
    channel = pwm_gpio_to_channel(pin);

    // Konfiguracja PWM dla 50Hz (standard dla serw)
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 64.0f); // Zegar systemowy dzielony przez 64
    pwm_config_set_wrap(&config, 39063);   // Ustawienie okresu na 20ms (50Hz)
    pwm_init(slice_num, &config, true);
}

void Servo::write(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Mapowanie kąta (0-180) na szerokość impulsu (500us - 2500us)
    // 500us to 0 stopni, 2500us to 180 stopni
    float pulse_width_us = 500.0f + (angle / 180.0f) * 2000.0f;
    
    // Konwersja czasu na poziom PWM (zegar po podziale to ~1.95MHz)
    uint16_t level = (uint16_t)(pulse_width_us * 1.953125f);
    pwm_set_chan_level(slice_num, channel, level);
}


// This function uses the hardware to create a unique seed
void seed_random_from_rosc() {
    uint32_t random = 0;
    volatile uint32_t *rnd_reg = (uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);
    
    // Mix bits from the hardware random generator
    for (int i = 0; i < 32; i++) {
        random = (random << 1) | (*rnd_reg & 1);
    }
    srand(random); // Initialize the standard random number generator
}

// Funkcja pomocnicza do odczytu czasu impulsu (zamiennik pulseIn)
uint64_t get_pulse_width(uint pin) {
    // Czekaj na stan wysoki (z timeoutem 100ms)
    absolute_time_t timeout = make_timeout_time_ms(30);
    while (gpio_get(pin) == 0) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) < 0) return 0;
    }
    absolute_time_t start_time = get_absolute_time();

    // Czekaj na stan niski
    while (gpio_get(pin) == 1) {
        if (absolute_time_diff_us(get_absolute_time(), timeout) < 0) return 0;
    }
    absolute_time_t end_time = get_absolute_time();

    return absolute_time_diff_us(start_time, end_time);
}


ServoPosition parallax_correction(const AimPosition &target_scanner) {
    constexpr float X_OFFSET_MM = 0.0f; 
    constexpr float Y_OFFSET_MM = 119.5f;
    constexpr float Z_OFFSET_MM = -18.75f;

    float dist_mm = target_scanner.DIST_CM * 10.0f;
    float alpha_rad = (target_scanner.HOR - DEFAULT_SCANNER_HOR_ANGLE) * 3.14159265f / 180.0f;    // pan (left/right)
    float beta_rad = (target_scanner.VER - DEFAULT_SCANNER_VER_ANGLE) * 3.14159265f / 180.0f;     // tilt (up/down)

    // XYZ position from scanner origin
    float x_s = dist_mm * cosf(beta_rad) * cosf(alpha_rad);
    float y_s = dist_mm * cosf(beta_rad) * sinf(alpha_rad);
    float z_s = dist_mm * sinf(beta_rad);

    // XYZ position from gun origin
    float x_g = x_s + X_OFFSET_MM;
    float y_g = y_s + Y_OFFSET_MM;
    float z_g = z_s + Z_OFFSET_MM;

    // Recalculate angles for gun
    float dist_g = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
    float alpha_g_rad = atan2f(y_g, x_g);
    float beta_g_rad = acosf(z_g / dist_g);

    // Convert back to degrees
    float alpha_g_deg = alpha_g_rad * 180.0f / 3.14159265f;
    float beta_g_deg = beta_g_rad * 180.0f / 3.14159265f;

    int gun_hor_angle = static_cast<int>(alpha_g_deg + DEFAULT_GUN_HOR_ANGLE);
    int gun_ver_angle = static_cast<int>(beta_g_deg + DEFAULT_GUN_VER_ANGLE);

    return {gun_ver_angle, gun_hor_angle};
}


void set_position(Servo &servo_ver, Servo &servo_hor, const ServoPosition &position) {
    servo_hor.write(position.HOR);
    sleep_ms(50);
    servo_ver.write(position.VER);
    sleep_ms(50);
}

float read_distance_cm() {
    // 1. Wyczyszczenie pinu TRIG
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);

    // 2. Wysłanie impulsu 10 mikrosekund
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    // 3. Odczyt czasu trwania sygnału (pulseIn)
    uint64_t duration = get_pulse_width(ECHO_PIN);

    // 4. Obliczenie odległości
    // 0.0343 cm/us
    float distance_cm = (float)duration * 0.0343f / 2.0f;

    led_on_if_in_range(distance_cm);

    return distance_cm;
}

void led_on_if_in_range(float distance_cm){
    if (MIN_SCANNER_DIST_CM <= distance_cm && distance_cm <= MAX_SCANNER_DIST_CM) {
        gpio_put(LED_PIN, 1);
    } else {
        gpio_put(LED_PIN, 0);
    }
}

void write_distance_to_buffer(const ServoPosition &scanner_servo, float distance_cm)
{
    static int buffer_index = 0;

    if (MIN_SCANNER_DIST_CM <= distance_cm && distance_cm <= MAX_SCANNER_DIST_CM) {
        scanBuffer[buffer_index].VER = scanner_servo.VER;
        scanBuffer[buffer_index].HOR = scanner_servo.HOR;
        scanBuffer[buffer_index].DIST_CM = distance_cm;
        
        buffer_index = (buffer_index + 1) % SCAN_BUFFER_SIZE;
    }
}

void write_distance_to_map(const ServoPosition &scanner_servo, float distance_cm) {

    if (MIN_SCANNER_DIST_CM <= distance_cm && distance_cm <= MAX_SCANNER_DIST_CM) {
        int v_idx = scanner_servo.VER;
        int h_idx = scanner_servo.HOR;
        if (v_idx >= 0 && v_idx <= ROW_NUM-1 && h_idx >= 0 && h_idx <= COL_NUM-1) {
            distanceMap[v_idx][h_idx] = distance_cm;
        }
    }
}

// ======== Ta wersja szuka celu w CAŁEJ mapie odległości ========
// AimPosition get_position_to_target() {
//         // Szukanie celu (najblizszy odczyt w tablicy)
//         float min_dist = 10000.0f;
//         int target_v = -1;
//         int target_h = -1;

//         for (int v = 0; v <= ROW_NUM-1; v++) {
//             for (int h = 0; h <= COL_NUM-1; h++) {
//                 float d = distanceMap[v][h];
//                 if (d > MIN_SCANNER_DIST_CM && d < MAX_SCANNER_DIST_CM) {
//                     if (d < min_dist) {
//                         min_dist = d;
//                         target_v = v;
//                         target_h = h;
//                     }
//                 }
//             }
//         }

//         return {target_v, target_h, min_dist};
// }

AimPosition get_position_to_target() {
    // Szukanie celu (najblizszy odczyt w buforze)
    float min_dist = 10000.0f;
    int target_v = -1;
    int target_h = -1;

    for (int i = 0; i < SCAN_BUFFER_SIZE; i++) {
        float d = scanBuffer[i].DIST_CM;
        if (d < min_dist) {
            min_dist = d;
            target_v = scanBuffer[i].VER;
            target_h = scanBuffer[i].HOR;
        }
    }

    return {target_v, target_h, min_dist};
}

void shoot() {
    // Symulacja strzału
    gpio_put(LASER_PIN, 1);
    gpio_put(BUZZER_PIN, 1);
    sleep_ms(1000); // Czas trwania strzału
    gpio_put(LASER_PIN, 0);
    gpio_put(BUZZER_PIN, 0);
}

void random_deg_pos(ServoPosition &scanner_servo) {
    scanner_servo.VER = rand() % (SCANNER_MAX_VER_ANGLE - SCANNER_MIN_VER_ANGLE + 1) + SCANNER_MIN_VER_ANGLE;
    scanner_servo.HOR = rand() % (SCANNER_MAX_HOR_ANGLE - SCANNER_MIN_HOR_ANGLE + 1) + SCANNER_MIN_HOR_ANGLE;
}

void random_scan_next_step(Servo &ver_servo, Servo &hor_servo, ServoPosition &scanner_servo){
    random_deg_pos(scanner_servo);
    set_position(ver_servo, hor_servo, scanner_servo);
}

void linear_scan_next_step(Servo &ver_servo, Servo &hor_servo, ServoPosition &scanner_servo) {
    static bool horizontalDirection = false; // true - prawo, false - lewo
    static bool verticalDirection = true;   // true - góra, false - dół
    bool move_vertical = false;

    if (horizontalDirection) {
        scanner_servo.HOR += 10;
        if (scanner_servo.HOR > SCANNER_MAX_HOR_ANGLE) {
            scanner_servo.HOR = SCANNER_MAX_HOR_ANGLE;
            horizontalDirection = false;
            move_vertical = true;
        }
    } else {
        scanner_servo.HOR -= 10;
        if (scanner_servo.HOR < SCANNER_MIN_HOR_ANGLE) {
            scanner_servo.HOR = SCANNER_MIN_HOR_ANGLE;
            horizontalDirection = true;
            move_vertical = true;
        }
    }

    if (move_vertical) {
        if (verticalDirection) {
            scanner_servo.VER += 10;
            if (scanner_servo.VER > SCANNER_MAX_VER_ANGLE) {
                scanner_servo.VER = SCANNER_MAX_VER_ANGLE;
                verticalDirection = false;
            }
        } else {
            scanner_servo.VER -= 10;
            if (scanner_servo.VER < SCANNER_MIN_VER_ANGLE) {
                scanner_servo.VER = SCANNER_MIN_VER_ANGLE;
                verticalDirection = true;
            }
        }
    }

<<<<<<< Updated upstream
    set_position(ver_servo, hor_servo, scanner_servo);
=======
    ver_servo.write(ver_pos_deg);
    sleep_ms(15);
    hor_servo.write(hor_pos_deg);
    sleep_ms(15);
}

// Obsługa zdarzeń Bluetooth
void bt_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    bd_addr_t event_addr;
    uint8_t rfcomm_channel_nr;
    uint16_t mtu;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case HCI_EVENT_PIN_CODE_REQUEST:
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    printf("SSP User Confirmation Request\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;

                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status 0x%02x\n", rfcomm_event_channel_opened_get_status(packet));
                        bt_connected = false;
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        printf("RFCOMM channel open succeeded. Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                        bt_connected = true;
                    }
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");
                    rfcomm_channel_id = 0;
                    bt_connected = false;
                    break;

                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            // Obsługa otrzymanych danych (jeśli potrzebna)
            printf("Received data: %.*s\n", size, packet);
            break;

        default:
            break;
    }
}

// Wysyłanie pomiarów przez Bluetooth w formacie "H,kąt,odległość" lub "V,kąt,odległość"
void bt_send_measurement(char axis, int angle, float distance) {
    if (!bt_connected || rfcomm_channel_id == 0) {
        return;
    }

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%c,%d,%.2f\n", axis, angle, distance);
    
    int err = rfcomm_send(rfcomm_channel_id, (uint8_t*)buffer, strlen(buffer));
    if (err) {
        printf("Error sending data via Bluetooth: %d\n", err);
    }
>>>>>>> Stashed changes
}