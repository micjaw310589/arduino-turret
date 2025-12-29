#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

// Definicje Pinów
#define ECHO_PIN 6
#define TRIG_PIN 7
#define LED_PIN 25         // Uwaga: Dioda na płytce Pico to zazwyczaj pin 25. Tu zostawiam 13 wg Twojego kodu.
#define LASER_PIN 8
#define SERVO_SENSOR_HOR_PIN 2
#define SERVO_SENSOR_VER_PIN 3
#define SERVO_GUN_HOR_PIN 4
#define SERVO_GUN_VER_PIN 5
#define BUZZER_PIN 9

// --- Klasa pomocnicza dla Serwomechanizmów (zamiast Servo.h) ---
class PicoServo {
    uint pin;
    uint slice_num;
    uint channel;

public:
    void attach(uint gpio_pin) {
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

    void write(int angle) {
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;

        // Mapowanie kąta (0-180) na szerokość impulsu (500us - 2500us)
        // 500us to 0 stopni, 2500us to 180 stopni
        float pulse_width_us = 500.0f + (angle / 180.0f) * 2000.0f;
        
        // Konwersja czasu na poziom PWM (zegar po podziale to ~1.95MHz)
        uint16_t level = (uint16_t)(pulse_width_us * 1.953125f);
        pwm_set_chan_level(slice_num, channel, level);
    }
};

// --- Globalne obiekty serw ---
PicoServo servo_sensor_hor;
PicoServo servo_sensor_ver;
PicoServo servo_gun_hor;
PicoServo servo_gun_ver;

// Zmienne pozycji
int pos_sensor_hor = 90;
int pos_sensor_ver = 90;
int pos_gun_hor = 90;
int pos_gun_ver = 90;

bool horizontalDirection = true; // true - prawo, false - lewo
bool verticalDirection = true;   // true - góra, false - dół

int maximumRange = 200;
int minimumRange = 20;
float distance;

// Mapa odległości [ver][hor] dla kątów 0-180 (indeks = kąt/10)
float distanceMap[181][181]; 

void fire_at_target(int ver_idx, int hor_idx) {
    int angle_ver = ver_idx;
    int angle_hor = hor_idx;

    // Wycelowanie
    servo_gun_ver.write(angle_ver);
    servo_gun_hor.write(angle_hor);
    
    // Symulacja strzału
    gpio_put(LASER_PIN, 1);
    gpio_put(BUZZER_PIN, 1);
    sleep_ms(100); // Czas trwania strzału
    gpio_put(LASER_PIN, 0);
    gpio_put(BUZZER_PIN, 0);
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

int main() {
    // Inicjalizacja Serial (USB)
    stdio_init_all();

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

    // Podłączenie serw
    servo_sensor_hor.attach(SERVO_SENSOR_HOR_PIN);
    servo_sensor_ver.attach(SERVO_SENSOR_VER_PIN);
    servo_gun_hor.attach(SERVO_GUN_HOR_PIN);
    servo_gun_ver.attach(SERVO_GUN_VER_PIN);

    // Ustawienie pozycji początkowych
    servo_sensor_hor.write(pos_sensor_hor);
    sleep_ms(300);
    servo_sensor_ver.write(pos_sensor_ver);
    sleep_ms(300);
    servo_gun_hor.write(pos_gun_hor);
    sleep_ms(300);
    servo_gun_ver.write(pos_gun_ver);
    sleep_ms(300);

    // Pętla główna (loop)
    while (true) {
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
        distance = (float)duration * 0.0343f / 2.0f;

        // Wyświetlenie wyniku
        printf("Zmierzona odległość: %.2f cm\n", distance);

        // Logika LED
        if (distance <= minimumRange || distance >= maximumRange) {
            // W oryginalnym kodzie było println(distance) tutaj też, ale to redundantne
            gpio_put(LED_PIN, 1);
        } else {
            gpio_put(LED_PIN, 0);
        }

        // Zapis do tablicy
        int v_idx = pos_sensor_ver;
        int h_idx = pos_sensor_hor;
        if (v_idx >= 0 && v_idx <= 180 && h_idx >= 0 && h_idx <= 180) {
             distanceMap[v_idx][h_idx] = distance;
        }

        // Szukanie celu
        float min_dist = 10000.0f;
        int target_v = -1;
        int target_h = -1;

        for (int i = 0; i <= 180; i++) {
            for (int j = 0; j <= 180; j++) {
                float d = distanceMap[i][j];
                if (d > minimumRange && d < maximumRange) {
                    if (d < min_dist) {
                        min_dist = d;
                        target_v = i;
                        target_h = j;
                    }
                }
            }
        }

        // Strzał
        if (target_v != -1 && target_h != -1) {
             fire_at_target(target_v, target_h);
             printf("Strzał w cel na odległości: %.2f cm (ver: %d, hor: %d)\n", min_dist, target_v, target_h);
        }else {   
            servo_gun_ver.write(pos_gun_ver);
            servo_gun_hor.write(pos_gun_hor);
            printf("Brak celu w zasięgu.\n");
        }

        // Delay 500ms
        //sleep_ms(500);

        // Logika poruszania sensorem (skanowanie 2D)
        bool move_vertical = false;

        if (horizontalDirection) {
            pos_sensor_hor += 10;
            if (pos_sensor_hor > 150) {
                pos_sensor_hor = 150;
                horizontalDirection = false;
                move_vertical = true;
            }
        } else {
            pos_sensor_hor -= 10;
            if (pos_sensor_hor < 30) {
                pos_sensor_hor = 30;
                horizontalDirection = true;
                move_vertical = true;
            }
        }

        if (move_vertical) {
            if (verticalDirection) {
                pos_sensor_ver += 10;
                if (pos_sensor_ver > 120) {
                    pos_sensor_ver = 120;
                    verticalDirection = false;
                }
            } else {
                pos_sensor_ver -= 10;
                if (pos_sensor_ver < 80) {
                    pos_sensor_ver = 80;
                    verticalDirection = true;
                }
            }
        }

        servo_sensor_hor.write(pos_sensor_hor);
        servo_sensor_ver.write(pos_sensor_ver);

        sleep_ms(15); // waits for the servo to reach the position
    }
    
    return 0;
}