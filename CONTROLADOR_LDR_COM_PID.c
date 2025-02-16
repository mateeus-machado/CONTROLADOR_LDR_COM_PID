#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define LED_PIN 13      // Pino de controle do LED
#define LDR_PIN 28      // Pino de leitura do LDR (ADC0)

// Coeficientes do PID
#define KP 1.2
#define KI 0.01
#define KD 0.5

// Variáveis do PID
float erro_anterior = 0;
float integral = 0;

// Setpoint (valor desejado do ADC)
const uint16_t SETPOINT = 1810;

void init_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255); // Define o valor máximo do PWM (8 bits)
    pwm_set_chan_level(slice, PWM_CHAN_A, 128); // Valor médio para brilho
    pwm_set_enabled(slice, true);
}

// Função PID para calcular a saída
uint8_t calcula_pid(uint16_t leitura_adc) {
    float erro = SETPOINT - leitura_adc;
    integral += erro;
    float derivativo = erro - erro_anterior;
    erro_anterior = erro;

    float saida = (KP * erro) + (KI * integral) + (KD * derivativo);
    
    // Limita a saída entre 10% e 90% do PWM (26 a 229)
    if (saida < 26) saida = 26;
    if (saida > 229) saida = 229;

    return (uint8_t)saida;
}

int main() {
    stdio_init_all();  // Inicializa comunicação serial
    init_pwm(LED_PIN); // Inicializa o PWM para controle do LED

    adc_init();              // Inicializa o ADC
    adc_gpio_init(LDR_PIN);  // Inicializa o pino do LDR

    while (1) {
        adc_select_input(0);  // Seleciona o canal 0 (pino 28)
        uint16_t leitura_adc = adc_read();  // Lê o valor do ADC

        // Calcula o PWM com o controle PID
        uint8_t pwm_value = calcula_pid(leitura_adc);

        // Ajusta o brilho do LED com o valor do PWM calculado
        pwm_set_gpio_level(LED_PIN, pwm_value);

        // Exibe a leitura no monitor serial
        printf("Leitura ADC: %u | PWM: %u\n", leitura_adc, pwm_value);

        sleep_ms(500);  // Aguarda 500ms antes de fazer a próxima leitura
    }
}
