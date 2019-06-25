/* Copyright 2017 Bolder Flight Systems <brian.taylor@bolderflight.com>.
 * Copyright 2018, Sergio Renato De Jesus Melean <sergiordj@gmail.com>.
 * Copyright 2018, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Pseudo code example: Reading RAM address 0x07 (Tobj1)
1. Send START bit
2. Send Slave Address (0x00* for example) + Rd\-Wr bit**
3. Send Command (0b000x_xxxx + 0b0000_0111 -> 0b0000_0111)
4. Send Repeated START_bit
5. Send Slave Address + Rd\-Wr bit**
6. Read Data Byte Low (master must send ACK bit)
7. Read Data Byte High (master must send ACK bit)
8. Read PEC (master can send ACK or NACK)
9. Send STOP bit */

/* Date: 2018-07-06 */

#ifndef _MLX90614_H_
#define _MLX90614_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
// MLX90614 registers                             UBICACION
#define MLX90614_TOBJ1_MAX          0X00          //EEPROM
#define MXL90614_TOBJ1 MIN          0x01          //EEPROM
#define MLX90614_RAWIR1             0x04          //RAM --- RAW TEMPERATURE SOURCE 1
#define MLX90614_RAWIR2             0x05          //RAM --- RAW TEMPERATURE SOURCE 2
#define MLX90614_TA                 0x06          //RAM --- LINEARIZED TEMPERATURE, AMBIENT
#define MLX90614_TOBJ1              0x07          //RAM --- LINEARIZED TEMPERATURE, SOURCE 1
#define MLX90614_TOBJ2              0x08          //RAM --- LINEARIZED TEMPERATURE, SOURCE 2
#define MLX90614_ToMAX              0x20          //EEPROM
#define MLX90614_ToMIN              0x21          //EEPROM
#define MLX90614_PWCTRL             0x22          //EEPROM
#define MLX90614_TARANGE            0x23          //EEPROM
#define MLX90614_EMISS              0x24          //EEPROM
#define MLX90614_CONFIG             0x25          //EEPROM
#define MLX90614_ADDR               0x0E          //EEPROM --- SMBUS Address
#define MLX90614_ID1                0x3C          //EEPROM
#define MLX90614_ID2                0x3D          //EEPROM
#define MLX90614_ID3                0x3E          //EEPROM
#define MLX90614_ID4                0x3F          //EEPROM
#define MLX90614_REGISTER SLEEP     0xFF


/** Read flags - bitmask. */
#define MLX90614_EEBUSY             0x80           //R/W flag bitmask - EEProm is busy (writing/erasing) */
#define MLX90614_EE_DEAD            0x20           //R/W flag bitmask - EEProm double error has occurred */
#define MLX90614_INIT               0x10           //R/W flag bitmask - POR initialization is still ongoing */

// Data bytes for temperature
#define DATALOW                     0x00
#define DATAHIGH                    0x00
#define PEC							0x00
#define tempFactor					0.02

// I2C baudrate
#define MLX90614_I2C_MINIMAL_RATE              10000 // 10  kHz
#define MLX90614_I2C_MAXIMAL_RATE             100000 // 100 kHz

// Timeout for read
#define I2C_READ_TIMEOUT            			1000 //Timeout para recibir el mensaje

//Mask to filter byte high. Byte high must be ignored to process the temperature
#define  MASK_HB					0x007F
/*==================[typedef]================================================*/

//

typedef enum {
MLX90614_ADDRESS_0  =  0x00,       //TODOS LOS ESCLAVOS RESPONDEN A ESTA DIRECCION
MLX90614_ADDRESS_1  =  0x5A        //FACTORY DEFAULT ADDRES // https://forum.arduino.cc/index.php?topic=54170.0
}MLX90614_address_t;

//Control structure for MLX90614 operation (only one IR per project)
typedef struct {

	MLX90614_address_t address;

	// buffer for reading from sensor
	uint8_t _buffer[2];

	// data buffer
	float _tempOffset;
	float _t;
	// track success of interacting with sensor
	int8_t _status;

} MLX90614_control_t;

typedef enum {
	MLX90614_1    = 0,          /* Gain =   1 Amplifier is bypassed*/
	MLX90614_3    = 1,          /* Gain =   3 */
	MLX90614_6    = 2,          /* Gain =   6 */
	MLX90614_125  = 3,          /* Gain =  12.5 */
	MLX90614_25   = 4,          /* Gain =  25 */
	MLX90614_50   = 5,          /* Gain =  50 */
	MLX90614_100  = 6,          /* Gain = 100 */
} MLX0614_gain_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef MLX90614_H_ */
