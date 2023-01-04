#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

#include "pico/multicore.h"


#include "pio_rotary_encoder1.pio.h"
#include "pio_rotary_encoder2.pio.h"

#define CMD_SET_VOLTAGE_1 101
#define CMD_SET_VOLTAGE_2 102
#define CMD_READ_VELOCITY_1 111
#define CMD_READ_VELOCITY_2 112

#define  BOARD_ID 1

const uint LED1_pin = 12;
const uint LED2_pin = 13;
const uint SW1_pin = 14;
const uint SW2_pin = 15;

const uint RS485_UART_TX_pin = 16;
const uint RS485_UART_RX_pin = 17;
const uint RS485_EN_pin = 26;

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

class RotaryEncoder
{
public:
    // constructor
    // rotary_encoder_A is the pin for the A of the rotary encoder.
    // The B of the rotary encoder has to be connected to the next GPIO.
    int port_num=0;
    RotaryEncoder(uint rotary_encoder_A, int _port_num)
    {
        port_num=_port_num;
        if(port_num==1){

            uint8_t rotary_encoder_B = rotary_encoder_A + 1;
            // pio 0 is used
            PIO pio = pio0;
            // state machine 0
            uint8_t sm = 0;
            // configure the used pins as input with pull up
            pio_gpio_init(pio, rotary_encoder_A);
            gpio_set_pulls(rotary_encoder_A, true, false);
            pio_gpio_init(pio, rotary_encoder_B);
            gpio_set_pulls(rotary_encoder_B, true, false);

            // load the pio program into the pio memory
            uint offset = pio_add_program(pio, &pio_rotary_encoder1_program);
            // make a sm config
            pio_sm_config c = pio_rotary_encoder1_program_get_default_config(offset);
            
            // set the 'in' pins
            sm_config_set_in_pins(&c, rotary_encoder_A);
            // set shift to left: bits shifted by 'in' enter at the least
            // significant bit (LSB), no autopush
            sm_config_set_in_shift(&c, false, false, 0);
            // set the IRQ handler
            irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
            // enable the IRQ
            irq_set_enabled(PIO0_IRQ_0, true);
            pio0_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS;
            // init the sm.
            // Note: the program starts after the jump table -> initial_pc = 16
            pio_sm_init(pio, sm, 16, &c);
            // enable the sm
            pio_sm_set_enabled(pio, sm, true);
        }else if(port_num==2){
            uint8_t rotary_encoder_B = rotary_encoder_A + 1;
            // pio 0 is used
            PIO pio = pio1;
            // state machine 0
            uint8_t sm = 1;
            // configure the used pins as input with pull up
            pio_gpio_init(pio, rotary_encoder_A);
            gpio_set_pulls(rotary_encoder_A, true, false);
            pio_gpio_init(pio, rotary_encoder_B);
            gpio_set_pulls(rotary_encoder_B, true, false);

            // load the pio program into the pio memory
            uint offset = pio_add_program(pio, &pio_rotary_encoder2_program);
            // make a sm config
            pio_sm_config c = pio_rotary_encoder2_program_get_default_config(offset);
            
            // set the 'in' pins
            sm_config_set_in_pins(&c, rotary_encoder_A);
            // set shift to left: bits shifted by 'in' enter at the least
            // significant bit (LSB), no autopush
            sm_config_set_in_shift(&c, false, false, 0);
            // set the IRQ handler
            irq_set_exclusive_handler(PIO1_IRQ_0, pio_irq_handler);
            // enable the IRQ
            irq_set_enabled(PIO1_IRQ_0, true);
            pio1_hw->inte0 = PIO_IRQ0_INTE_SM0_BITS | PIO_IRQ0_INTE_SM1_BITS;
            // init the sm.
            // Note: the program starts after the jump table -> initial_pc = 16
            pio_sm_init(pio, sm, 16, &c);
            // enable the sm
            pio_sm_set_enabled(pio, sm, true);

        }
    }

    // set the current rotation to a specific value
    void set_rotation(int _rotation)
    {
        if(port_num==1)rotation1 = _rotation;
        if(port_num==2)rotation2 = _rotation;
    }

    // get the current rotation
    int get_rotation(void)
    {
        if(port_num==1)return rotation1;
        if(port_num==2)return rotation2;
        return 0;
    }

private:
    static void pio_irq_handler()
    {
        // test if irq 0 was raised
        if (pio0_hw->irq & 1)
        {
            rotation1 = rotation1 - 1;
            pio0_hw->irq = 3;
        }
        // test if irq 1 was raised
        if ((pio0_hw->irq & 2))
        {
            rotation1 = rotation1 + 1;
            pio0_hw->irq = 3;
        }
        if((pio1_hw->irq & 1)){
            rotation2 = rotation2 - 1;
            pio1_hw->irq = 3;
        }
        if((pio1_hw->irq & 2)){
            rotation2 = rotation2 + 1;
            pio1_hw->irq = 3;
        }
        // clear both interrupts
    }

    // the pio instance
    PIO pio;
    // the state machine
    uint sm;
    // the current location of rotation
    static int rotation1;
    static int rotation2;
};

int RotaryEncoder::rotation1 = 0;
int RotaryEncoder::rotation2 = 0;

static semaphore_t sem;
static int g_motor1_taeget_velocity;
static int g_motor2_taeget_velocity;
static int g_motor1_taeget_current;
static int g_motor2_taeget_current;
static int g_encorder1_velocity;
static int g_encorder2_velocity;
static int g_encorder1_position;
static int g_encorder2_position;


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

    gpio_init(RS485_EN_pin);
    gpio_set_dir(RS485_EN_pin, GPIO_OUT);
    gpio_put(RS485_EN_pin,0);
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
    if(v<0){
        gpio_put(motor1_EN1_pin,1);
        gpio_put(motor1_EN2_pin,1);
        pwm_set_chan_level(motor1_PWM1_slice_num, PWM_CHAN_B, 0);
        pwm_set_chan_level(motor1_PWM2_slice_num, PWM_CHAN_A,-v);
    }else{
        gpio_put(motor1_EN1_pin,1);
        gpio_put(motor1_EN2_pin,1);
        pwm_set_chan_level(motor1_PWM1_slice_num, PWM_CHAN_B, v);
        pwm_set_chan_level(motor1_PWM2_slice_num, PWM_CHAN_A, 0);

    }
}

void SetVoltage2(int v){
    if(v<0){
        gpio_put(motor2_EN1_pin,1);
        gpio_put(motor2_EN2_pin,1);
        pwm_set_chan_level(motor2_PWM1_slice_num, PWM_CHAN_B, 0);
        pwm_set_chan_level(motor2_PWM2_slice_num, PWM_CHAN_A,-v);
    }else{
        gpio_put(motor2_EN1_pin,1);
        gpio_put(motor2_EN2_pin,1);
        pwm_set_chan_level(motor2_PWM1_slice_num, PWM_CHAN_B, v);
        pwm_set_chan_level(motor2_PWM2_slice_num, PWM_CHAN_A, 0);
    }
}


void core1_main(){
    uart_init(uart0,115200);
    gpio_set_function(RS485_UART_TX_pin, GPIO_FUNC_UART);
    gpio_set_function(RS485_UART_RX_pin, GPIO_FUNC_UART);

    while(1){
        int vel1_ref=0;
        int vel2_ref=0;
        int enc1_pos=0;
        int enc2_pos=0;
        bool set_vel_flag=false;
        sem_acquire_blocking(&sem);
        enc1_pos=g_encorder1_position;
        enc2_pos=g_encorder2_position;
        sem_release(&sem);

        gpio_put(RS485_EN_pin,0);
        while(uart_is_readable(uart0)){
            if(int(uart_getc(uart0))==BOARD_ID){
                gpio_put(LED2_pin, 1);
                switch(int(uart_getc(uart0))){
                    case CMD_READ_VELOCITY_1:
                        gpio_put(RS485_EN_pin,1);
                        printf("HEAD:%d,%d\n", enc1_pos,  enc2_pos);
                        //printf("HEAD:%d,%d\n",  gpio_get(SW1_pin),gpio_get(SW2_pin));
                        break;
                    case CMD_READ_VELOCITY_2:
                        gpio_put(RS485_EN_pin,1);
                        printf("HEAD:%d,%d\n",  gpio_get(SW1_pin),gpio_get(SW2_pin));
                        break;
                    case CMD_SET_VOLTAGE_1:
                        gpio_put(LED1_pin, 1);

                        vel1_ref = int(uart_getc(uart0));
                        set_vel_flag=true;
                        gpio_put(RS485_EN_pin,1);
                        printf("HEAD:%d,%d\n",  gpio_get(SW1_pin),gpio_get(SW2_pin));

                        break;
                    case CMD_SET_VOLTAGE_2:
                        vel2_ref  = int(uart_getc(uart0));
                        set_vel_flag=true;

                        gpio_put(RS485_EN_pin,1);
                        printf("HEAD:%d,%d\n",  gpio_get(SW1_pin),gpio_get(SW2_pin));
                        break;
                    default:
                        gpio_put(RS485_EN_pin,1);
                        printf("ERROR:%d,%d\n",  gpio_get(SW1_pin),gpio_get(SW2_pin));
                        break;
                }
            }
        }
        gpio_put(LED2_pin, 0);
        if(set_vel_flag){
            sem_acquire_blocking(&sem);
            g_motor1_taeget_velocity=vel1_ref;
            g_motor2_taeget_velocity=vel2_ref;
            sem_release(&sem);
        }
        
        sleep_ms(10);

    }
}

int main()
{
    stdio_init_all();
//    setup_default_uart();
    init();

    RotaryEncoder encoder1(4,1);
    RotaryEncoder encoder2(6,2);
    encoder1.set_rotation(0);
    encoder2.set_rotation(0);

    sem_init(&sem, 1, 1);

    multicore_launch_core1(core1_main);

    while (1)
    {
        int v=0;
        gpio_put(LED1_pin, 0);
        gpio_put(LED2_pin, 0);
        int nn=-123;
        int vel1=0;
        int vel2=0;

        sem_acquire_blocking(&sem);
        g_encorder1_position=encoder1.get_rotation();
        g_encorder2_position=encoder2.get_rotation();
        vel1=g_motor1_taeget_velocity;
        vel2=g_motor2_taeget_velocity;
        sem_release(&sem);

        if(gpio_get(SW1_pin)){
            SetVoltage1(vel1);
            gpio_put(LED1_pin, 1);
        }else{
            SetVoltage1(0);
            gpio_put(LED1_pin, 0);
        }
        if(gpio_get(SW1_pin))SetVoltage2(vel2);
        else SetVoltage2(0);

        sleep_ms(1);
        // for(int i=-50;i<50;i++){
        //     if(!gpio_get(SW1_pin))SetVoltage1(i);
        //     else SetVoltage1(0);
        //     if(!gpio_get(SW2_pin))SetVoltage2(i);
        //     else SetVoltage2(0);
        //     gpio_put(LED1_pin,gpio_get(SW1_pin));
        //     gpio_put(LED2_pin,gpio_get(SW2_pin));
    	//     gpio_put(RS485_EN_pin,1);
        //     printf("%d,%d,%d\n",i,  gpio_get(SW1_pin),gpio_get(SW2_pin));
        //     sleep_ms(10);
        // }
        // for(int i=50;i>-50;i--){
        //     if(!gpio_get(SW1_pin))SetVoltage1(i);
        //     else SetVoltage1(0);
        //     if(!gpio_get(SW2_pin))SetVoltage2(i);
        //     else SetVoltage2(0);
        //     gpio_put(LED1_pin,gpio_get(SW1_pin));
        //     gpio_put(LED2_pin,gpio_get(SW2_pin));
	    //     gpio_put(RS485_EN_pin,1);
        //     printf("%d,%d,%d\n",i,  gpio_get(SW1_pin),gpio_get(SW2_pin));
	    //     gpio_put(RS485_EN_pin,1);
        //     sleep_ms(10);
        // }
    }
}
