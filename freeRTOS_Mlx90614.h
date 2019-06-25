/*=====[Nombre del modulo]=====================================================
 * Copyright 2019 Fabian sarmiento  <fsarmiento1805@gmail.com>
 * All rights reserved.
 * Licencia: Texto de la licencia o al menos el nombre y un link
         (ejemplo: BSD-3-Clause <https://opensource.org/licenses/BSD-3-Clause>)
 *
 * Version: 0.0.0
 * Fecha de creacion: YYYY/MM/DD
 */

/*=====[Evitar inclusion multiple comienzo]==================================*/
#ifndef _freeRTOS_Mlx90614_H_
#define _freeRTOS_Mlx90614_
/*=====[Inclusiones de dependencias de funciones publicas]===================*/
// Includes de FreeRTOS
#include "FreeRTOS.h"   //Motor del OS
#include "FreeRTOSConfig.h"
#include "task.h"		//Api de control de tareas y temporización
#include "semphr.h"		//Api de sincronización (sem y mutex)
#include "queue.h"      //Api de colas

// sAPI header
#include "sapi.h"
#include "board.h"

// Includes sAPI
#include "mlx90614.h"           /* <=  MLX90614 header */
#include "sapi_i2c.h"           /* <= sAPI I2C header */
#include "sapi_delay.h"         /* <= sAPI Delay header */

/*=====[C++ comienzo]========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Macros de definicion de constantes publicas]=========================*/

#define UP      1
#define FALLING 2
#define DOWN	3
#define RISING  4

#define CANT_TECLAS 2
#define CANT_LEDS 2
#define ANTI_BOUNCE_TIME_MS 40
/*=====[Macros estilo funcion publicas]======================================*/

/*=====[Definiciones de tipos de datos publicos]=============================*/

// Tipo de datos que renombra un tipo basico

// Tipo de datos de puntero a funcion

// Tipo de datos enumerado
enum Teclas_t {Tecla1, Tecla2}; //Índices de teclas para el vector de estructuras


// Tipo de datos estructura, union o campo de bits
struct Button_Control { //estructura de control de datos capturados por la interrupción
	portTickType TiempoCaptura;
	uint8_t Flanco;
};

struct Buttons_SM_t{ //estructura de control de la máquina de estados de cada botón
	uint8_t Indice;
	uint8_t Estado;
	xQueueHandle Cola;
	portTickType TiempoIntermedio;
};

struct Lectura_t{
	uint8_t Indice;
	portTickType TiempoMedido;
};

//Definición de vector de estructuras de control
struct Buttons_SM_t Buttons_SM[CANT_TECLAS];

/*=====[Prototipos de funciones publicas]====================================*/

// Prototipo de funcion de la tarea
void Tecla( void* taskParmPtr );

void Led_task( void* taskParmPtr );

void MedicionT(void* taskParmPtr );

void MensajesT(void* taskParmPtr );

/*=====[Prototipos de funciones publicas de interrupcion]====================*/
//Función de inicialización de interrupciones
void My_IRQ_Init (void);

/*=====[C++ fin]=============================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Evitar inclusion multiple fin]=======================================*/
#endif
