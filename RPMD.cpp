#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

const uint LED1_pin = 12;
const uint LED2_pin = 13;
const uint SW1_pin = 14;
const uint SW2_pin = 15;

const uint motor1_PWM1_pin = 25; //PWM4 B
const uint motor1_EN1_pin = 24;
const uint motor1_PWM2_pin = 22; //PWM3 A	
const uint motor1_EN2_pin = 23;

const uint motor2_PWM1_pin = 21; // PWM2 B	
const uint motor2_EN1_pin = 20;
const uint motor2_PWM2_pin = 18; //PWM1 A	
const uint motor2_EN2_pin = 19;

uint motor1_PWM1_slice_num;
uint motor1_PWM2_slice_num;
uint motor2_PWM1_slice_num;
uint motor2_PWM2_slice_num;

void init(){
    gpio_init(LED1_pin);
    gpio_set_dir(LED1_pin, GPIO_OUT);


    gpio_init(LED2_pin);
    gpio_set_dir(LED2_pin, GPIO_OUT);

    gpio_init(SW1_pin);
    gpio_set_dir(SW1_pin, GPIO_IN);
    gpio_pull_up(SW1_pin);

    gpio_init(SW2_pin);
    gpio_set_dir(SW2_pin, GPIO_IN);
    gpio_pull_up(SW2_pin);

// Motor1
    gpio_init(motor1_PWM1_pin);
    gpio_set_dir(motor1_PWM1_pin, GPIO_OUT);
    gpio_set_function(motor1_PWM1_pin, GPIO_FUNC_PWM);
    motor1_PWM1_slice_num = pwm_gpio_to_slice_num(motor1_PWM1_pin);

    gpio_init(motor1_EN1_pin);
    gpio_set_dir(motor1_EN1_pin, GPIO_OUT);

    gpio_init(motor1_PWM2_pin);
    gpio_set_dir(motor1_PWM2_pin, GPIO_OUT);
    gpio_set_function(motor1_PWM2_pin, GPIO_FUNC_PWM);
    motor1_PWM2_slice_num = pwm_gpio_to_slice_num(motor1_PWM2_pin);

    gpio_init(motor1_EN2_pin);
    gpio_set_dir(motor1_EN2_pin, GPIO_OUT);

// Motor2
    gpio_init(motor2_PWM1_pin);
    gpio_set_dir(motor2_PWM1_pin, GPIO_OUT);
    gpio_set_function(motor2_PWM1_pin, GPIO_FUNC_PWM);
    motor2_PWM1_slice_num = pwm_gpio_to_slice_num(motor2_PWM1_pin);

    gpio_init(motor2_EN1_pin);
    gpio_set_dir(motor2_EN1_pin, GPIO_OUT);

    gpio_init(motor2_PWM2_pin);
    gpio_set_dir(motor2_PWM2_pin, GPIO_OUT);
    gpio_set_function(motor2_PWM2_pin, GPIO_FUNC_PWM);
    motor2_PWM2_slice_num = pwm_gpio_to_slice_num(motor2_PWM2_pin);

    gpio_init(motor2_EN2_pin);
    gpio_set_dir(motor2_EN2_pin, GPIO_OUT);


    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_clkdiv(motor1_PWM1_slice_num,125);//125MHzをn分周
    pwm_set_wrap(motor1_PWM1_slice_num, 100-1);
    pwm_set_phase_correct(motor1_PWM1_slice_num,true);//三角波
    // Set channel A output high for one cycle before dropping
    // Set the PWM running
    pwm_set_enabled(motor1_PWM1_slice_num, true);

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_clkdiv(motor1_PWM2_slice_num,125);//125MHzをn分周
    pwm_set_wrap(motor1_PWM2_slice_num, 100-1);
    pwm_set_phase_correct(motor1_PWM2_slice_num,true);//三角波
    // Set channel A output high for one cycle before dropping
    // Set the PWM running
    pwm_set_enabled(motor1_PWM2_slice_num, true);

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_clkdiv(motor2_PWM1_slice_num,125);//125MHzをn分周
    pwm_set_wrap(motor2_PWM1_slice_num, 100-1);
    pwm_set_phase_correct(motor2_PWM1_slice_num,true);//三角波
    // Set channel A output high for one cycle before dropping
    // Set the PWM running
    pwm_set_enabled(motor2_PWM1_slice_num, true);

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_clkdiv(motor2_PWM2_slice_num,125);//125MHzをn分周
    pwm_set_wrap(motor2_PWM2_slice_num, 100-1);
    pwm_set_phase_correct(motor2_PWM2_slice_num,true);//三角波
    // Set channel A output high for one cycle before dropping
    // Set the PWM running
    pwm_set_enabled(motor2_PWM2_slice_num, true);



}
void SetVoltage1(int v){
    gpio_put(motor1_EN1_pin,1);
    gpio_put(motor1_EN2_pin,1);
    pwm_set_chan_level(motor1_PWM1_slice_num, PWM_CHAN_B, v);
    pwm_set_chan_level(motor1_PWM2_slice_num, PWM_CHAN_A, 0);

}

void SetVoltage2(int v){
    gpio_put(motor2_EN1_pin,1);
    gpio_put(motor2_EN2_pin,1);
    pwm_set_chan_level(motor2_PWM1_slice_num, PWM_CHAN_B, v);
    pwm_set_chan_level(motor2_PWM2_slice_num, PWM_CHAN_A, 0);

}

int main()
{
    stdio_init_all();
    setup_default_uart();

    init();

    while (1)
    {
        for(int i=0;i<50;i++){
            SetVoltage1(i);
            SetVoltage2(i);
            gpio_put(LED1_pin,gpio_get(SW1_pin));
            gpio_put(LED2_pin,gpio_get(SW2_pin));
            //printf("%d,  %d\n",  gpio_get(SW1_pin),gpio_get(SW2_pin));
            sleep_ms(10);
        }
        for(int i=50;i>0;i--){
            SetVoltage1(i);
            SetVoltage2(i);
            gpio_put(LED1_pin,gpio_get(SW1_pin));
            gpio_put(LED2_pin,gpio_get(SW2_pin));
            //printf("%d,  %d\n",  gpio_get(SW1_pin),gpio_get(SW2_pin));
            sleep_ms(10);
        }
    }
}