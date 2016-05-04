#ifndef GPIO_F4_H_
#define GPIO_F4_H_

#include "platform.h"

/* Definy poszczegolnych trybow dzialania portu GPIO */
#define GPIO_MODE_INPUT       0
#define GPIO_MODE_OUTPUT      1
#define GPIO_MODE_AF          2
#define GPIO_MODE_ANALOG      3

/**
 * @brief Funkcja ustawia tryb na pinie GPIO.
 *
 * @param gpio                         Port do ustawienia.
 * @param pin                          Pin do ustawienia.
 * @param conf                         Tryb do ustawienia.
 */
void gpio_mode_config(GPIO_TypeDef * gpio, int32_t pin, int32_t conf);

/**
 * @brief Ustawienie pinu GPIO w trybie push-pull.
 *
 * @param gpio                         Port do ustawienia.
 * @param pin                          Pin do ustawienia.
 */
#define gpio_otype_pp_set(gpio, pin)   gpio->OTYPER &= ~(1<<pin);

/**
 * @brief Ustawienie pinu GPIO w trybie open drain.
 *
 * @param gpio                         Port do ustawienia.
 * @param pin                          Pin do ustawienia.
 */
#define gpio_otype_od_set(gpio, pin)   gpio->OTYPER |= (1<<pin);

/* Definy poszczegolnych predkosci portu GPIO */
#define GPIO_SPEED_LOW        0
#define GPIO_SPEED_MEDIUM     1
#define GPIO_SPEED_FAST       2
#define GPIO_SPEED_HIGH       3

/**
 * @brief Funkcja ustawia predkosc pinu GPIO.
 *
 * @param gpio                         Port do ustawienia.
 * @param pin                          Pin do ustawienia.
 * @param conf                         Predkosc do ustawienia.
 */
void gpio_speed_config(GPIO_TypeDef * gpio, int32_t pin, int32_t conf);

/* Definy poszczegolnych wartosci pull-up, pull-down */
#define GPIO_PUPD_NONE        0
#define GPIO_PUPD_PU          1
#define GPIO_PUPD_PD          2
#define GPIO_PUPD_RESERVED    3

/**
 * @brief Funkcja ustawia pull-up pull-down pinu GPIO.
 *
 * @param gpio                         Port do ustawienia.
 * @param pin                          Pin do ustawienia.
 * @param conf                         Konfiguracja pull-up do ustawienia.
 */
void gpio_pupd_config(GPIO_TypeDef * gpio, int32_t pin, int32_t conf);

/* Definy poszczegolnych wartosci funkcji alternatywnych portu. */
#define GPIO_AF_SYS           0
#define GPIO_AF_TIM1          1
#define GPIO_AF_TIM2          1
#define GPIO_AF_TIM3          2
#define GPIO_AF_TIM4          2
#define GPIO_AF_TIM5          2
#define GPIO_AF_TIM8          3
#define GPIO_AF_TIM9          3
#define GPIO_AF_TIM10         3
#define GPIO_AF_TIM11         3
#define GPIO_AF_I2C1          4
#define GPIO_AF_I2C2          4
#define GPIO_AF_I2C3          4
#define GPIO_AF_SPI1          5
#define GPIO_AF_SPI2          5
#define GPIO_AF_I2S2          5
#define GPIO_AF_I2S2EXT       5
#define GPIO_AF_SPI3          6
#define GPIO_AF_I2SEXT        6
#define GPIO_AF_I2S3          6
#define GPIO_AF_USART1        7
#define GPIO_AF_USART2        7
#define GPIO_AF_USART3        7
#define GPIO_AF_I2S3EXT       7
#define GPIO_AF_UART4         8
#define GPIO_AF_UART5         8
#define GPIO_AF_USART6        8
#define GPIO_AF_CAN1          9
#define GPIO_AF_CAN2          9
#define GPIO_AF_TIM12         9
#define GPIO_AF_TIM13         9
#define GPIO_AF_TIM14         9
#define GPIO_AF_OTGFS1        10
#define GPIO_AF_OTGHS         10
#define GPIO_AF_ETH           11
#define GPIO_AF_FSMC          12
#define GPIO_AF_SDIO          12
#define GPIO_AF_OTGFS2        12
#define GPIO_AF_DCMI          13
#define GPIO_AF_AF14          14
#define GPIO_AF_EVENTOUT      15

/**
 * @brief Funkcja ustawia funkcje alternatywna dla pinu GPIO.
 *
 * @param gpio                         Port do ustawienia.
 * @param pin                          Pin do ustawienia.
 * @param conf                         Funkcja alternatywna do ustawienia.
 */
void gpio_af_config(GPIO_TypeDef * gpio, int32_t pin, int32_t conf);

#endif /* GPIO_F4_H_ */
