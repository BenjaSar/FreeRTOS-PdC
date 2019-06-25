/*==================[inlcusiones]============================================*/
#include "freeRTOS_Mlx90614.h"
/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

SemaphoreHandle_t Mutex_uart;  		//Mutex que protege la UART de concurrencia
SemaphoreHandle_t MedicionfromISR;  // Semaforo que me permite iniciar la medicion una vez pulsada la tecla

TaskHandle_t myMedicionTaskHandle = NULL;
TaskHandle_t myMensajesTaskHandle = NULL;


xQueueHandle Cola_Lecturas;
xQueueHandle Cola_Medicion;
xQueueHandle Cola_MensajesT;

TickType_t TickstoWait = 100;

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
	// Declaracion de variables que indican el estado del sistema
	bool_t status;
	uint8_t Error_state = 0;
	// ---------- CONFIGURACIONES ------------------------------
	// Inicializar y configurar la plataforma
	boardConfig();

	//Iniciamos las interrupciones
	My_IRQ_Init();



	// Inicializar I2C para comunicacion con el  mxl90614 IR
	status =  i2cInit(I2C0, MLX90614_I2C_MAXIMAL_RATE );

	// UART for debug messages
	debugPrintConfigUart( UART_USB, 115200 );
	printf( "Prototipo dispositivo sensor de ejes calientes implementado con freeRTOS.\r\n" );

	printf("Inicializando MLX90614...\r\n" );

	// Verificacion del arranque del Sistema -- Mensaje de error en caso de falla
	if( status < 0 ){
		printf( "MLX90614 no inicializado, por favor chequear las conexiones:\r\n\r\n" );
		printf( "MLX90614 ---- EDU-CIAA-NXP\r\n\r\n" );
		printf( "    VCC ---- 3.3V\r\n" );
		printf( "    GND ---- GND\r\n" );
		printf( "    SCL ---- SCL\r\n" );
		printf( "    SDA ---- SDA\r\n" );
		printf( "Error del Sistema.\r\n" );
		gpioToggle(LEDR);
		while(1);
	}

	/* Creamos colas de capturas de teclas */
	int8_t i;
	for (i = CANT_TECLAS ; i-- ; i >= 0) {
		Buttons_SM[i].Indice = i;
		if (NULL == (Buttons_SM[i].Cola = xQueueCreate(5,sizeof(struct Button_Control)))){
			Error_state =1;
		}
	}

	/* Creamos cola de lecturas completadas */
	if (NULL == (Cola_Lecturas = xQueueCreate(10,sizeof(struct Lectura_t)))){
		Error_state =1;
	}

	if (NULL == (Mutex_uart = xSemaphoreCreateMutex())){
		Error_state =1;
	}

	// Creamos cola de datos medidos
	if (NULL == (Cola_MensajesT = xQueueCreate(10,sizeof(float )))){
		Error_state =1;
	}

	//Creacion del semaforo para interactuar con la ISR
	if ( (MedicionfromISR = xSemaphoreCreateBinary())== NULL ){
		Error_state = 1;
	}

	// Crear tareas de Teclas en freeRTOS
	xTaskCreate(
			Tecla,                     // Funcion de la tarea a ejecutar
			(const char *)"Tec1",     // Nombre de la tarea como String amigable para el usuario
			configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
			&Buttons_SM[0],                 // Parametros de tarea
			tskIDLE_PRIORITY+1,         // Prioridad de la tarea
			0                           // Puntero a la tarea creada en el sistema
	);

	xTaskCreate(Tecla,(const char *)"Tec2",configMINIMAL_STACK_SIZE*2, &Buttons_SM[1],
			tskIDLE_PRIORITY+1,
			0
	);

	// Crear tarea LED en freeRTOS
	xTaskCreate(Led_task,(const char *)"Led", configMINIMAL_STACK_SIZE*2, 0,
			tskIDLE_PRIORITY+1,
			0
	);

	// Crear tarea para efectuar la medicion de T
	xTaskCreate(
			MedicionT, (const char *)"Medicion T", configMINIMAL_STACK_SIZE*6, 0,
			tskIDLE_PRIORITY+1,
			myMedicionTaskHandle
	);

	// Tarea que permite la impresion en pantalla a traves de la UART
	xTaskCreate(
			MensajesT, (const char *)"Mensajes", configMINIMAL_STACK_SIZE*4, 0,
			tskIDLE_PRIORITY+1,
			myMensajesTaskHandle
	);


	// Iniciar scheduler
	if (Error_state==0){
		vTaskStartScheduler();
	} else{
		printf("Error al iniciar el sistema !");
		gpioWrite (LEDR, ON);
	}

	// ---------- REPETIR POR SIEMPRE --------------------------
	while( TRUE ) {
		// Si cae en este while 1 significa que no pudo iniciar el scheduler
	}

	// NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
	// directamenteno sobre un microcontroladore y no es llamado por ningun
	// Sistema Operativo, como en el caso de un programa para PC.
	return 0;
}

/*==================[definiciones de funciones internas]=====================*/
//Función de inicialización de IRQs
void My_IRQ_Init (void){
	//Inicializamos las interrupciones (LPCopen)
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	//Inicializamos cada evento de interrupción (LPCopen)
	Chip_SCU_GPIOIntPinSel(0, 0, 4); //Mapeo del pin donde ocurrirá el evento y
	//el canal al que lo va a enviar. (Canal 0 a 7, Puerto GPIO, Pin GPIO)
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);//Se configura el canal
	//para que se active por flanco
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);//Se configura para que el
	//flanco sea el de bajada

	Chip_SCU_GPIOIntPinSel(1, 0, 4);//En este caso el canal de interrupción es 1
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH1);//En este caso el flanco es
	//de subida


	Chip_SCU_GPIOIntPinSel(2, 0, 8);
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH2);
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH2);

	Chip_SCU_GPIOIntPinSel(3, 0, 8);
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH3);
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH3);

	/* Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);*/


	//Una vez que se han configurado los eventos para cada canal de interrupcion
	//Se activan las interrupciones para que comiencen a llamar al handler
	NVIC_SetPriority(PIN_INT0_IRQn, 2);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_SetPriority(PIN_INT1_IRQn, 2);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
	NVIC_SetPriority(PIN_INT2_IRQn, 2);
	NVIC_EnableIRQ(PIN_INT2_IRQn);
	NVIC_SetPriority(PIN_INT3_IRQn, 2);
	NVIC_EnableIRQ(PIN_INT3_IRQn);

	/*NVIC_EnableIRQ(I2C0_IRQn);*/
}

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion de la tarea genérica Tecla
void Tecla( void* taskParmPtr )
{
	// ---------- Definición de variables locales ------------------------------
	struct Buttons_SM_t* Maq_estado; //Me preparo para recibir la dirección de la estructura y copiarla en una varibale local
	Maq_estado = (struct Buttons_SM_t*) taskParmPtr;
	Maq_estado->Estado = UP;
	struct Lectura_t Lectura;
	Lectura.Indice = Maq_estado->Indice;

	struct Button_Control Snapshot;
	portTickType Last_Snapshot = 0;

	// ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE) {

		if (xQueueReceive(Maq_estado->Cola, &Snapshot, portMAX_DELAY)){

			switch (Maq_estado->Estado){

			case UP:
				if(Snapshot.Flanco == FALLING){ //Acá adentro está el pseudo estado Falling

					if (pdFALSE == (xQueueReceive(Maq_estado->Cola, &Snapshot, (ANTI_BOUNCE_TIME_MS / portTICK_RATE_MS)))){
						Maq_estado->Estado = DOWN;

						//Acá se mete código para ejecutar en flanco  de bajada
						Maq_estado->TiempoIntermedio = Snapshot.TiempoCaptura;

					}
				}
				break;

			case DOWN:
				if(Snapshot.Flanco == RISING){ //Acá adentro está el pseudo estado Rising

					if (pdFALSE == (xQueueReceive(Maq_estado->Cola, &Snapshot, (ANTI_BOUNCE_TIME_MS / portTICK_RATE_MS)))){
						Maq_estado->Estado = UP;

						//Acá se mete código para ejecutar en flanco  de subida
						Lectura.TiempoMedido = xTaskGetTickCount() - Maq_estado->TiempoIntermedio;
						xQueueSend(Cola_Lecturas, &Lectura, portMAX_DELAY);
					}
				}
				break;

			default:
				Maq_estado->Estado = UP;
				break;

			}

		}


	}

}


// Implementacion de funcion de la tarea Led
void Led_task( void* taskParmPtr ){

	struct Lectura_t Lectura;

	gpioMap_t Led_Map[CANT_LEDS] = {LED1,LED2};

	while (TRUE){
		//Espero evento de Lectura completada
		if (xQueueReceive(Cola_Lecturas, &Lectura, portMAX_DELAY)){

			gpioWrite(Led_Map[Lectura.Indice],ON);

			//Espero tiempo de encendido
			vTaskDelay( Lectura.TiempoMedido );

			gpioWrite(Led_Map[Lectura.Indice],OFF);
		}
	}
}

//Implementacion de la tarea medicion
void MedicionT(void* taskParmPtr ){

	uint8_t transmitbufferT[3];    		//Buffer del registro de temperatura del Objeto
	uint8_t  bufferToReadTO[2];	  		// Espacio para guardar los bytes de lectura de la Tobjeto linearizada
	uint8_t  bufferToReadTA[2];	  		// Espacio para guardar los bytes de lectura de la Tambiente linearizada

	float TemperaturaObjeto1;           //Valor de Temperatura del objeto calculado
	float TemperaturaObjeto2;           //Valor de Temperatura calculado
	float TemperaturaAmbiente;          //Valor de Temperatura Ambiente calculado


	portTickType xMedicionTr = 500/portTICK_RATE_MS;
	portTickType xLastWakeTimer = xTaskGetTickCount();

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Sistema iniciado de manera correcta. Led G indicador de inicio del Sistema
	if( Mutex_uart != NULL ){
		if (xSemaphoreTake( Mutex_uart,TickstoWait  )==pdTRUE  ){


			printf("MLX90614  se ha inicializado correctamente.\r\n\r\n" );
			gpioWrite (LEDG, ON);
			vTaskDelay( 2000 / portTICK_RATE_MS );
			gpioWrite (LEDG, OFF);
			vTaskDelay( 2000 / portTICK_RATE_MS );

			xSemaphoreGive( Mutex_uart );

		}
	}
	/* ------------- REPETIR POR SIEMPRE ------------- */
	while(TRUE){
		if( xSemaphoreTake( MedicionfromISR, portMAX_DELAY )==pdTRUE){


			//Leer el sensor y guardar en estructura de control
			transmitbufferT[0] = MLX90614_TOBJ1;
			transmitbufferT[1] = MLX90614_TOBJ2;
			transmitbufferT[2] = MLX90614_TA;

			//Buffer de datos obtenidos para la temperatura del Objeto
			bufferToReadTO[0]= DATALOW;
			bufferToReadTO[1]= DATAHIGH;
			/*bufferToReadTO[2]= PEC;  //PEC*/

			//Buffer de datos obtenidos para la temperatura Ambiente
			bufferToReadTA[0]= DATALOW;
			bufferToReadTA[1]= DATAHIGH;
			/*bufferToReadTA[2]= 0xE1;  //PEC*/
			//Medicion de temperatura ambiente y objeto - Tarea periodica cada xMedicionTr
			i2cRead( I2C0,MLX90614_ADDRESS_1, transmitbufferT,1,FALSE, bufferToReadTO,2,FALSE);
			vTaskDelayUntil(  &xLastWakeTimer, xMedicionTr );
			i2cRead( I2C0, MLX90614_ADDRESS_1, &transmitbufferT[2],1,FALSE, bufferToReadTA,2,FALSE);
			vTaskDelayUntil(  &xLastWakeTimer, xMedicionTr );


			// Senial de vida de la lectura de i2c
			if (i2cRead){
				gpioToggle (LEDG);}

			//Calculo de la Temperatura Objeto y la temperatura Ambiente
			TemperaturaObjeto1 = ((((bufferToReadTO[1])<<8)+bufferToReadTO[0])*tempFactor)-273.15;
			vTaskDelay( 150 / portTICK_RATE_MS );
			/*TemperaturaObjeto1 = ((((bufferToReadTO[1])<<8)+bufferToReadTO[0])*tempFactor)-273.15;*/
			TemperaturaAmbiente = (((bufferToReadTA[1]<<8)+bufferToReadTA[0])*tempFactor)-273.15;
			vTaskDelay( 300 / portTICK_RATE_MS );
			/*printf( "Temperatura Objeto1:   %d  [C]\r\n\r\n", TemperaturaObjeto);*/
			printf( "Temperatura Ambiente:   %0.2f  [C]\r\n\r\n", TemperaturaAmbiente);
			xQueueSend(Cola_MensajesT, &TemperaturaObjeto1, xMedicionTr);

		}

	}

}

void MensajesT(void* taskParmPtr ){
	float TemperaturaObjeto1;            //Valor de Temperatura calculado
	/* ------------- REPETIR POR SIEMPRE ------------- */

	while (TRUE){

		if (xQueueReceive(Cola_MensajesT, &TemperaturaObjeto1, ( TickType_t ) 1000)){

			// Imprimir  de la variable T calculada en la funcion medicion
			printf( "Temperatura Objeto1:  %0.2f  [C]\r\n\r\n", TemperaturaObjeto1);
			//Impresion mensaje de emergencia
			if (TemperaturaObjeto1 >=100){
				printf("Exceso de Temperatura: %0.2f [C]\r\n\r\n", TemperaturaObjeto1);

			}

		}
	}
}

void GPIO0_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; //Comenzamos definiendo la variable


	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH0){ //Verificamos que la interrupción es la esperada
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0); //Borramos el flag de interrupción
		//codigo a ejecutar si ocurrió la interrupción

		struct Button_Control Snapshot;
		Snapshot.Flanco = FALLING;
		Snapshot.TiempoCaptura = xTaskGetTickCountFromISR();

		xQueueSendFromISR( Buttons_SM[Tecla1].Cola, &Snapshot, &xHigherPriorityTaskWoken );
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO1_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t xHigherPriorityTask = pdFALSE;
	uint8_t transmitbufferT[3];   //Buffer de los registros a transmistir


	if (Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH1){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
		//codigo a ejecutar si ocurrió la interrupción
		struct Button_Control Snapshot;
		Snapshot.Flanco = RISING;
		Snapshot.TiempoCaptura = xTaskGetTickCountFromISR();
		xQueueSendFromISR( Buttons_SM[Tecla1].Cola, &Snapshot, &xHigherPriorityTaskWoken );
		xSemaphoreGiveFromISR(MedicionfromISR, &xHigherPriorityTask);
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTask );

}

void GPIO2_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; //Comenzamos definiendo la variable


	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH2){ //Verificamos que la interrupción es la esperada
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH2); //Borramos el flag de interrupción
		//codigo a ejecutar si ocurrió la interrupción
		struct Button_Control Snapshot;
		Snapshot.Flanco = FALLING;
		Snapshot.TiempoCaptura = xTaskGetTickCountFromISR();
		xQueueSendFromISR( Buttons_SM[Tecla2].Cola, &Snapshot, &xHigherPriorityTaskWoken );
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void GPIO3_IRQHandler(void){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH3){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH3);
		//codigo a ejecutar si ocurrió la interrupción
		struct Button_Control Snapshot;
		Snapshot.Flanco = RISING;
		Snapshot.TiempoCaptura = xTaskGetTickCountFromISR();
		xQueueSendFromISR( Buttons_SM[Tecla2].Cola, &Snapshot, &xHigherPriorityTaskWoken );
	}
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/*==================[fin del archivo]========================================*/
