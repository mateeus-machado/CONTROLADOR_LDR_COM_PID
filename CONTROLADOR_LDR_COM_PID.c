#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define LED_PIN 13      // Pino de controle do LED
#define LDR_PIN 28      // Pino de leitura do LDR (ADC0)

// Função para mapear o valor do ADC para o intervalo de PWM desejado
uint8_t map_adc_to_pwm(uint16_t adc_value, uint16_t adc_min, uint16_t adc_max, uint8_t pwm_min, uint8_t pwm_max) {
    // Mapeia o valor do ADC para o intervalo do PWM (10% a 90%)
    return (adc_value - adc_min) * (pwm_max - pwm_min) / (adc_max - adc_min) + pwm_min;
}

void init_pwm(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255); // Define o valor máximo do PWM (8 bits)
    pwm_set_chan_level(slice, PWM_CHAN_A, 128); // Valor médio para brilho
    pwm_set_enabled(slice, true);
}

int main() {
    stdio_init_all();  // Inicializa comunicação serial
    init_pwm(LED_PIN); // Inicializa o PWM para controle do LED

    adc_init();              // Inicializa o ADC
    adc_gpio_init(LDR_PIN);  // Inicializa o pino do LDR

    // Faixa de valores do ADC e valores de PWM (10% a 90%)
    const uint16_t ADC_MIN = 1800; // Valor mínimo do ADC
    const uint16_t ADC_MAX = 1820; // Valor máximo do ADC
    const uint8_t PWM_MIN = 26;    // 10% de 255 (aproximadamente)
    const uint8_t PWM_MAX = 229;   // 90% de 255 (aproximadamente)

    while (1) {
        adc_select_input(0);  // Seleciona o canal 0 (pino 28)
        uint16_t leitura_adc = adc_read();  // Lê o valor do ADC

        // Mapeia o valor do ADC para o intervalo de PWM
        uint8_t pwm_value = map_adc_to_pwm(leitura_adc, ADC_MIN, ADC_MAX, PWM_MIN, PWM_MAX);

        // Ajusta o brilho do LED com o valor do PWM calculado
        pwm_set_gpio_level(LED_PIN, pwm_value);

        // Exibe a leitura no monitor serial
        printf("Leitura ADC: %u | PWM: %u\n", leitura_adc, pwm_value);

        sleep_ms(500);  // Aguarda 500ms antes de fazer a próxima leitura
    }
}
