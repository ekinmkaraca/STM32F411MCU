/*
 * st32f411xx_gpio_driver.h
 *
 *  Created on: Feb 12, 2022
 *      Author: Ekin
 */

#ifndef INC_ST32F411XX_GPIO_DRIVER_H_
#define INC_ST32F411XX_GPIO_DRIVER_H_

#include <stdint.h>
#include "st32f411xx.h"

namespace gpio{
    enum class mode {
        input = 0b00, output = 0b01, altfun = 0b10, analog = 0b11
    };
    enum class output {
        PP = 0b0, OD = 0b1
    };
    enum class speed {
        low = 0b00, medium = 0b01, fast = 0b10, high = 0b11
    };
    enum class pupd {
        nopupd = 0b00, pu = 0b01, pd=0b10
    };
    enum class altfun:int {
         af0 = 0, af1, af2, af3, af4, af5, af6, af7, af8, af9, af10, af11, af12, af13, af14, af15
    };
    enum class pins:int {
	pin0 = 0, pin1, pin2, pin3, pin4, pin5, pin6, pin7, pin8, pin9, pin10, pin11, pin12, pin13, pin14, pin15
    };

    enum class trigger{
    	Rising = 0, Falling, RisingFalling
    };

    struct Pin{
        uint8_t pin;
        GPIO_RegDef_t* port;
    };


    inline void init(Pin pin, mode m = mode::input, output t = output::PP, speed s = speed::low, pupd p = pupd::nopupd, altfun a = altfun::af0);
    inline void init(Pin pin, mode m = mode::input, pupd p = pupd::nopupd, altfun a = altfun::af0);
    inline void set(Pin pin);
    inline void reset(Pin pin);
    inline void toggle(Pin pin);
    inline uint8_t read(Pin pin);
    inline void write(Pin pin,uint8_t value);
    inline void IRQ_config(Pin pin, trigger tr);
    inline void IRQ_read(Pin pin);
    inline void IRQ_handler(uint8_t pinNum);


};


inline void gpio::init(Pin pin, mode m, pupd p, altfun a){
    GPIO_RegDef_t* port_temp ;
    port_temp = GPIO_A;
    if (pin.port == port_temp)
        GPIOA_PCLK_EN();
    port_temp = GPIO_B;
    if (pin.port == port_temp)
        GPIOB_PCLK_EN();
    port_temp = GPIO_C;
    if (pin.port == port_temp)
        GPIOC_PCLK_EN();
    port_temp = GPIO_D;
    if (pin.port == port_temp)
        GPIOD_PCLK_EN();
    port_temp = GPIO_E;
    if (pin.port == port_temp)
        GPIOE_PCLK_EN();
    port_temp = GPIO_H;
    if (pin.port == port_temp)
        GPIOH_PCLK_EN();

    uint32_t temp1 = 0, temp2 = 0;
    temp1 |= (0b11 << (2 * (int)pin.pin));
    pin.port->MODER &= ~(temp1);                    //Clear the reg
    temp1 = 0;
    temp1 |= int(m) << ((2 * (int)pin.pin));
    pin.port->MODER |= temp1;                       //Set the reg
    temp1 = 0;

    temp1 |= 0b11 << ((2 * (int)pin.pin));
    pin.port->PUPDR &= ~(temp1);
    temp1 = 0;
    temp1 |= int(p) << ((2 * (int)pin.pin));
    pin.port->PUPDR |= temp1;
    temp1 = 0;

    (int(pin.pin) < 8 ? temp1 : temp2) |= (0b11 << (4 * (int(pin.pin) - (int(pin.pin) < 8 ? 0 : 8))));
    pin.port->AFR[0] &= ~temp1;
    pin.port->AFR[1] &= ~temp2;
    temp1 = 0;
    temp2 = 0;
    (int(pin.pin) < 8 ? temp1 : temp2) |= (int(a) << (4 * (int(pin.pin) - (int(pin.pin) < 8 ? 0 : 8))));
    pin.port->AFR[0] |= temp1;
    pin.port->AFR[1] |= temp2;
}

inline void gpio::init(Pin pin, mode m, output t, speed s, pupd p, altfun a) {
    GPIO_RegDef_t* port_temp ;
    port_temp = GPIO_A;
    if (pin.port == port_temp)
        GPIOA_PCLK_EN();
    port_temp = GPIO_B;
    if (pin.port == port_temp)
        GPIOB_PCLK_EN();
    port_temp = GPIO_C;
    if (pin.port == port_temp)
        GPIOC_PCLK_EN();
    port_temp = GPIO_D;
    if (pin.port == port_temp)
        GPIOD_PCLK_EN();
    port_temp = GPIO_E;
    if (pin.port == port_temp)
        GPIOE_PCLK_EN();
    port_temp = GPIO_H;
    if (pin.port == port_temp)
        GPIOH_PCLK_EN();

    uint32_t temp1 = 0, temp2 = 0;
    temp1 |= (0b11 << (2 * (int)pin.pin));
    pin.port->MODER &= ~(temp1);                    //Clear the reg
    temp1 = 0;
    temp1 |= int(m) << ((2 * (int)pin.pin));
    pin.port->MODER |= temp1;                       //Set the reg
    temp1 = 0;

    if((int)m != 0 ){
		temp1 |= (0b1 << (2 * (int)pin.pin));
		pin.port->OTYPER &= ~(temp1);
		temp1 = 0;
		temp1 |= int(t) << (int)pin.pin;
		pin.port->OTYPER |= temp1;
		temp1 = 0;


		temp1 |= (0b11 << (2 * (int)pin.pin));
		pin.port->OSPEEDR &= ~(temp1);
		temp1 = 0;
		temp1 |= int(s) << ((2 * (int)pin.pin));
		pin.port->OSPEEDR |= temp1;
		temp1 = 0;
    }

    temp1 |= 0b11 << ((2 * (int)pin.pin));
    pin.port->PUPDR &= ~(temp1);
    temp1 = 0;
    temp1 |= int(p) << ((2 * (int)pin.pin));
    pin.port->PUPDR |= temp1;
    temp1 = 0;

    (int(pin.pin) < 8 ? temp1 : temp2) |= (0b11 << (4 * (int(pin.pin) - (int(pin.pin) < 8 ? 0 : 8))));
    pin.port->AFR[0] &= ~temp1;
    pin.port->AFR[1] &= ~temp2;
    temp1 = 0;
    temp2 = 0;
    (int(pin.pin) < 8 ? temp1 : temp2) |= (int(a) << (4 * (int(pin.pin) - (int(pin.pin) < 8 ? 0 : 8))));
    pin.port->AFR[0] |= temp1;
    pin.port->AFR[1] |= temp2;
}

inline void gpio::set(Pin pin){
    pin.port->BSRR = 0b1 << pin.pin;
}

inline void gpio::reset(Pin pin){
    pin.port->BSRR = 0b1 << (pin.pin+16);
}

inline void gpio::toggle(Pin pin){
    pin.port->ODR^=(1<<(pin.pin));
}

inline uint8_t gpio::read(Pin pin){
    uint8_t value;
    value = (uint8_t)((pin.port->IDR >> int(pin.pin)) & 0x00000001);
    return value;
}

inline void gpio::write(Pin pin, uint8_t value){
    if (value == 0)
        pin.port->ODR &= ~(1 << pin.pin);
    else
        pin.port->ODR |= (1 << pin.pin);
}

inline void gpio::IRQ_config(Pin pin, trigger tr){
	SYSCFG_PCLK_EN();
	uint32_t* SYSCFG_EXTI_ADD = ((uint32_t*)(SYSCFG->EXTICR + (int(pin.pin)/4)*4));

	uint32_t temp = (pin.port->MODER >> ((2 * (int)pin.pin))) & 0x00000003; /*Is pin mode input?*/
	if( temp != 0x00)
		pin.port->MODER &= ~(0b1 << ((2 * (int)pin.pin)));

	GPIO_RegDef_t* port_temp ;
	port_temp = GPIO_A;
	if (pin.port == port_temp)
		*SYSCFG_EXTI_ADD |= (0b0000 << (int(pin.pin))%4*4);
	port_temp = GPIO_B;
	if (pin.port == port_temp)
		*SYSCFG_EXTI_ADD |= (0b0001 << (int(pin.pin))%4*4);
	port_temp = GPIO_C;
	if (pin.port == port_temp)
		*SYSCFG_EXTI_ADD |= (0b0010 << (int(pin.pin))%4*4);
	port_temp = GPIO_D;
	if (pin.port == port_temp)
		*SYSCFG_EXTI_ADD |= (0b0011 << (int(pin.pin))%4*4);;
	port_temp = GPIO_E;
	if (pin.port == port_temp)
		*SYSCFG_EXTI_ADD |= (0b0100 << (int(pin.pin))%4*4);
	port_temp = GPIO_H;
	if (pin.port == port_temp)
		*SYSCFG_EXTI_ADD |= (0b0111 << (int(pin.pin))%4*4);

	EXTI->IMR |= (0b1 << (int)pin.pin);
	EXTI->EMR |= (0b1 << (int)pin.pin);

	if(tr == trigger::Rising)
		EXTI->RTSR |= (0b1 << (int)pin.pin);
	if(tr == trigger::Falling)
		EXTI->FTSR |= (0b1 << (int)pin.pin);
	if(tr == trigger::RisingFalling){
		EXTI->FTSR |= (0b1 << (int)pin.pin);
		EXTI->RTSR |= (0b1 << (int)pin.pin);
	}

	int extiNum = 0;
	if((int)pin.pin >= 0 && (int)pin.pin <= 4){
		extiNum = (int)pin.pin + 6 ;
	}
	else if((int)pin.pin >= 5 && (int)pin.pin <= 9){
		extiNum = 23;
	}
	else if((int)pin.pin >= 10 && (int)pin.pin <= 15){
		extiNum = 40;
	}


	uint32_t* NVIC_ISER = (uint32_t*)(NVIC_ISER_BASEADDR + ((extiNum)/32)*4);
	*NVIC_ISER |= (0b1 << extiNum);
}

void gpio::IRQ_handler(uint8_t pinNum){
	if(EXTI->PR & (1 << pinNum)){
		EXTI->PR |= 1 << pinNum;
	}

}


#endif /* INC_ST32F411XX_GPIO_DRIVER_H_ */
