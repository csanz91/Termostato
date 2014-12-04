#include "utility/u8g.h"
#include "U8glib.h"
#include "rtc.h"
#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"



U8GLIB_ST7920_128X64_1X u8g(40,42,43);
//U8GLIB_ST7920_128X64_1X(sck, mosi, cs [, reset])


// Hardware configuration: Set up nRF24L01 radio on SPI bus uplus pins CE & CS 
RF24 radio(7,8);

/*  ----------------------------------------------------------------------------------------
 *  Constants
 *  ----------------------------------------------------------------------------------------
 */
 


#define PRESCALER_TIEMPO 1	//8X5=40 segs
#define PRESCALER_TEMP 1	//8X2=16 segs
#define PRESCALER_BOTONES 30
#define PRESCALER_RADIADOR 2
#define PRESCALER_LCD	6
#define TEMP_OFFSET 3
#define MARGEN_TEMP 0.4
#define NUM_PROGS	2	//PROGRAMAS TOTALES: SUMARLE UNO
#define INCREMENTO_PROG 15
#define NUM_ENVIOS_RC 6
#define NUM_MAX_MUESTRAS 99
int PRESCALER_PUL_LARGA= 100;
#define NUM_PIPES		 4
#define PRESCALER_ENVIO	 1
#define MAX_TIEMPO_ENVIO 400

#define STATION			 0

#define DATOS_GRAFICA	 5

#define OFF				 0
#define MANUAL			 1
#define PROGRAMADO_ON	 2
#define PROGRAMADO_OFF	 3

#define MENU_PPL		 0
#define CAMBIAR_HORA	 3
#define ESTADISTICAS	 1
#define PROGRAMACION	 2
#define NUM_MENUS		 3
#define NUM_CURSOR_HORA  4

#define HORAS			 1
#define TEMP			 2			 
#define PROG			 3

/*  ----------------------------------------------------------------------------------------
 *  Pin Assignments
 *  ----------------------------------------------------------------------------------------
 */


#define	pinSubir			PK1 // Boton subir
#define	pinBotones			PINK
#define	pinBajar			PK0 // Boton bajar
#define	pinIzqd				PK3 // Boton izquierda
#define	pinDrcha			PK4 // Boton derecha
#define	pinOk				PK2 // Boton ok
#define	pinONOFF			PL1 // Boton bajar
#define portONOFF			PORTL
#define ddrONOFF			DDRL
#define PINONOFF 			PINL
#define portLcdBacklight	PORTL
#define ddrLcdBacklight		DDRL
#define pinLcdBacklight		PL0

/*  ----------------------------------------------------------------------------------------
 *  Global variables
 *  ----------------------------------------------------------------------------------------
 */

int  temp_min=9999, temp_max, temp_media, temperaturas[99];
char  menu_seleccionado, 
 cursor_prog_edicion, cursor_prog_seleccion, flag_subir_prog, flag_bajar_prog, cursor_prog_arriba,
 cursor_prog_abajo, contador_actualizar_botones, cursor_cambio_hora, flag_subir_hora, flag_bajar_hora,
 flag_subir_larga, flag_subir_corta, cursor_temperaturas, envio_OK, selected_station_envio=0, selected_station=0;
 
int contador_subir_larga, contador_bajar_larga;

char volatile contador_actualizar_tiempo, flag_actualizar_tiempo=1, contador_actualizar_temp,
 flag_actualizar_temp=1, contador_actualizar_radiador, flag_actualizar_radiador=1, flag_subir, flag_bajar,
 flag_izqd, flag_drcha, flag_ok , flag_encender_pantalla, contador_encender_pantalla, flag_recibido=0, recepcion_enable,
 contador_envio, flag_envio, contador_espera_recepcion, intento_envios;

// Topology
	const uint64_t pipes[NUM_STATIONS][6] = { {0xAAAAAAAAAA00, 0xAAAAAAAAAA01, 0xAAAAAAAAAA02, 0xAAAAAAAAAA03, 0xAAAAAAAAAA04, 0xAAAAAAAAAA05},
											  {0xBBBBBBBBBB00, 0xBBBBBBBBBB01, 0xBBBBBBBBBB02, 0xBBBBBBBBBB03, 0xBBBBBBBBBB04, 0xBBBBBBBBBB05},
											  {0xCCCCCCCCCC00, 0xCCCCCCCCCC01, 0xCCCCCCCCCC02, 0xCCCCCCCCCC03, 0xCCCCCCCCCC04, 0xCCCCCCCCCC05},
											  {0xDDDDDDDDDD00, 0xDDDDDDDDDD01, 0xDDDDDDDDDD02, 0xDDDDDDDDDD03, 0xDDDDDDDDDD04, 0xDDDDDDDDDD05},
											  {0xEEEEEEEEEE00, 0xEEEEEEEEEE01, 0xEEEEEEEEEE02, 0xEEEEEEEEEE03, 0xEEEEEEEEEE04, 0xBBBBBBBBBB05}
											  };     // Radio pipe addresses for the 2 nodes to communicate.


struct data stations[NUM_STATIONS];

int counter;


/*  ----------------------------------------------------------------------------------------
 *  LCD Setup
 *  ----------------------------------------------------------------------------------------
  
	K (backlight cathode)  GND
	A (backlight annode)   +5V
	PSD (SPI Mode)          GND (SEE NOTE)
	E (SCK)                PORTB, Pin 5
	R/W (MOSI)             PORTB, Pin 3
	RS (CS)                PORTB, Pin 4
	VDD                    +5V
	VSS                    GND
 */


/*  ----------------------------------------------------------------------------------------
 *  Prototypes
 *  ----------------------------------------------------------------------------------------
 */






/*  ----------------------------------------------------------------------------------------
 *  set up functions
 *  ----------------------------------------------------------------------------------------
 */



void set_up_timer()
{
	//Initialize Timer1
	TCCR1A = 0;				 // set entire TCCR1A register to 0
	TCCR1B = 0;				 // same for TCCR1B
	// set compare match register to desired timer count:
	OCR1A = 62500;			//8Mhz/(1024)-1 -> 8s
	// turn on CTC mode:
	TCCR1B |= (1 << WGM12);
	// Set CS10 and CS12 bits for 1024 prescaler:
	TCCR1B |=  (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt:
	TIMSK1 |= (1 << OCIE1A);
}

void set_up_io()
{
	//Configuracion interrupciones
	PCICR |= (1 << PCIE2);  //PCINT16..23
	PCMSK2|= (1 << PCINT16) | (1 << PCINT20) | (1 << PCINT19) | (1 << PCINT18) | (1 << PCINT17);
	
	//ddrONOFF  |= (1<<pinONOFF);
	//portONOFF   |= (1<<pinONOFF);
	
	ddrLcdBacklight |= (1<<pinLcdBacklight);
	portLcdBacklight |= (1<<pinLcdBacklight);

}

void set_up_nRF24L01(){


	

 // Setup and configure rf radio
 	printf("\n\rRF24/examples/GettingStarted/\n\r");
 	radio.begin();
	radio.setAutoAck(1);                    // Ensure autoACK is enabled
	radio.setDataRate(RF24_250KBPS);
	radio.setRetries(15,15);                 // Smallest time between retries, max no. of retries
	//radio.enableDynamicPayloads();                // Here we are sending 1-byte payloads to test the call-response speed
	radio.setPayloadSize(32); 
  radio.openReadingPipe(1,pipes[0][0]);
  radio.openReadingPipe(2,pipes[0][1]);
  radio.openReadingPipe(3,pipes[0][2]);
  radio.openReadingPipe(4,pipes[0][3]);

	// for(int y=0;y<NUM_STATIONS;y++){                      //NO FUNCIONA
	// 	for(int x=1;x<NUM_PIPES;x++){
	// 		 radio.openReadingPipe(x,pipes[y][x-1]);
	// 	}
	// }
   
	radio.startListening();                 // Start listening
	radio.printDetails();                   // Dump the configuration of the rf unit for debugging
}

void send(unsigned char payload[]){

	byte gotByte;
	long tiempo_inicial=millis();
	
	while((millis()-tiempo_inicial)<900){
		if(radio.write( payload, 32)){
			    flag_envio=0;
			    break;
		}else{
			printf("failed.\n\r");
			delay(10);
		}	
	}
}



void Send_Data(uint8_t addr, struct data stations[])
{
	radio.stopListening();                                  // First, stop listening so we can talk.
	unsigned char payload[33];

	for(selected_station_envio=1; selected_station_envio<2; selected_station_envio++){

	 	if (stations[selected_station_envio].cambios&(1<<HORAS))
	 	{
	 		//sprintf(payload, "%d%d", data2send->hour, data2send->minute);
			radio.openWritingPipe(pipes[selected_station_envio][0]);
			payload[0]=stations[selected_station_envio].hour;
			payload[1]=stations[selected_station_envio].minute;
			send(payload);

		}
		if (stations[selected_station_envio].cambios&(1<<TEMP))
	 	{

			radio.openWritingPipe(pipes[selected_station_envio][1]);
			
			payload[0]=stations[selected_station_envio].temperatura/100;
			payload[1]=stations[selected_station_envio].temperatura%100;
			payload[2]=stations[selected_station_envio].temp_prog/100;
			send(payload);

		}
		if (stations[selected_station_envio].cambios&(1<<PROG))
	 	{
			
			radio.openWritingPipe(pipes[selected_station_envio][2]);
			for(int i=0; i<4; i++){
			    payload[i]=stations[selected_station_envio].prog_1[i];
			}
			for(int i=0; i<4; i++){
			    payload[i+4]=stations[selected_station_envio].prog_2[i];
			}
			for(int i=0; i<4; i++){
			    payload[i+8]=stations[selected_station_envio].prog_3[i];
			}
			send(payload);

		}
		stations[selected_station_envio].cambios=0;

		Receive_Data(selected_station_envio, &stations[selected_station_envio]);
	}
}


void Receive_Data(uint8_t raddress, struct data* data2receive){

   
  radio.openReadingPipe(1,pipes[raddress][0]);
  radio.openReadingPipe(2,pipes[raddress][1]);
  radio.openReadingPipe(3,pipes[raddress][2]);
  radio.openReadingPipe(4,pipes[raddress][3]);
  radio.startListening();
  delay(200);

   byte i;                  
   unsigned char gotByte[33];                                      // Dump the payloads until we've gotten everything

	while( radio.available(&i)){
		radio.read( &gotByte, 32);
		if(i==1){

			data2receive->hour=gotByte[0];
			data2receive->minute=gotByte[1];
		}else if(i==2){
			data2receive->temperatura=gotByte[0]*100+gotByte[1];
			data2receive->temp_prog=gotByte[2]*100;
		}else if(i==3){
			for(int i=0; i<4; i++){
		   		 data2receive->prog_1[i]=gotByte[i];
			}
			for(int i=0; i<4; i++){
			    data2receive->prog_2[i]=gotByte[i+4];
			}
			for(int i=0; i<4; i++){
			    data2receive->prog_3[i]=gotByte[i+8];
			}
		}
	}
}



/*  ----------------------------------------------------------------------------------------
 *  uiMain functions
 *  ----------------------------------------------------------------------------------------
 */


void uiMainDrawTemp(struct data data2draw)
{
	char buffer[4];
	u8g.setFont(u8g_font_fub25);
	sprintf(buffer, "%2.0d", data2draw.temperatura/100);
	u8g.drawStr(3, 30, buffer);
	u8g.setFont(u8g_font_fub14n);
	sprintf(buffer, ".%1d", (data2draw.temperatura/10)%10);
	u8g.drawStr(39, 28, buffer);
}

void uiMainDrawStation()
{
	char buffer[2];
	sprintf(buffer, "%d", selected_station);
	u8g.drawStr(0, 64, buffer);
}

void uiMainDrawTrend(struct data data2draw)
{
	u8g.setFont(u8g_font_6x10);
	if (data2draw.temperatura/10>data2draw.temp_anterior/10)
	{
		u8g.drawTriangle(71, 0, 81, 15, 61, 15);
		if (data2draw.activado) u8g.drawStr(66 , 27, "ON");
		else u8g.drawStr(63 , 27, "OFF");
	
	}else
	{
		u8g.drawTriangle(71, 30, 81, 15, 61, 15);
		if (data2draw.activado) u8g.drawStr(66 , 10, "ON");
		else u8g.drawStr(63 , 10, "OFF");
	}
	
}

void uiMainDrawSetTemp(int temp_prog)
{
	char buffer[4];
	u8g.setFont(u8g_font_fub25);
	sprintf(buffer, "%2d", temp_prog/100);
	u8g.drawStr(83, 30, buffer);
}

void uiMainDrawStatus(char estado)
{

	switch (estado) {
		case OFF:
			u8g.drawStr(6, 52, "Apagado");
		break;
		case MANUAL:
			u8g.drawStr(6, 52, "Manual");
		break;
		case PROGRAMADO_ON:
			u8g.drawStr(6, 52, "Program ON");
		break;
		case PROGRAMADO_OFF:
			u8g.drawStr(6, 52, "Program OFF");
		break;
		default:
			u8g.drawStr(6, 52, "Error");
	}
}

void uiMainDrawHTime(struct data data2draw)
{
	char buffer[5];
	u8g.setFont(u8g_font_fub14n);
	sprintf(buffer, "%02d:%02d", data2draw.hour, data2draw.minute);
	u8g.drawStr(70, 55, buffer);
}

void uiMain(struct data data2draw)
{
	
	u8g.drawLine(6, 32, 120, 32); //Seperator
	
	uiMainDrawTemp(data2draw);
	uiMainDrawTrend(data2draw);
	uiMainDrawStatus(data2draw.estado);
	uiMainDrawStation();
	if (data2draw.estado) uiMainDrawSetTemp(data2draw.temp_prog);
	uiMainDrawHTime(data2draw);
	
}


/*  ----------------------------------------------------------------------------------------
 *  uiStats functions
 *  ----------------------------------------------------------------------------------------
 */

void uiStats()
{
	u8g.setFont(u8g_font_6x10);
	u8g.drawStr(6, 10, "Estadisticas");
	u8g.drawLine(6, 12, 120, 12); //Seperator
	uiStatsTemps();
	uiStatsGraph();
	
}

void uiStatsTemps()
{




	char buffer[6];
	temp_media=1234;
	sprintf(buffer, "Med:%2.0d", temp_media/100);
	u8g.drawStr(6, 40, buffer);
	sprintf(buffer, "Max:%2.0d", temp_max/100);
	u8g.drawStr(6, 25, buffer);
	
	sprintf(buffer, "Min:%2.0d", temp_min/100);
	u8g.drawStr(6, 55, buffer);
}

void uiStatsGraph()
{
	unsigned char coodenadaX1, coordenadaY1, coordenadaX2, coordenadaY2;
	u8g.drawLine(60, 18, 60, 55);
	u8g.drawLine(60, 55, 120, 55);
	
	u8g.setFont(u8g_font_4x6);
	u8g.drawStr(52, 38, "20");
	u8g.drawStr(52, 23, "30");
	u8g.drawStr(52, 53, "10");
	u8g.drawStr(77, 62, "5 horas");
	
	
	coodenadaX1=61;
	coordenadaY1=-floor((temperaturas[0]/100)*1.5)+65;
	if (coordenadaY1>55) coordenadaY1=55;
	else if (coordenadaY1<15) coordenadaY1=15;
	
	for (int x=1; x<=DATOS_GRAFICA; x++)
	{
		if((temperaturas[x*10])>1000){
		    
		
		coordenadaX2=x*6+60;
		coordenadaY2=-floor((temperaturas[x*10]/100)*1.5)+65;
		if (coordenadaY2>55) coordenadaY2=55;
		else if (coordenadaY2<15) coordenadaY2=15;
		u8g.drawLine(coodenadaX1, coordenadaY1, coordenadaX2, coordenadaY2);
		coodenadaX1=coordenadaX2;
		coordenadaY1=coordenadaY2;
		}
	}
}

/*  ----------------------------------------------------------------------------------------
 *  uiSetTime functions
 *  ----------------------------------------------------------------------------------------
 */
void uiSetTimeShowTime(struct data data2show)
{
	char buffer[19];
	sprintf(buffer, "%02d  %02d  %02d  %02d  %02d", data2show.year, data2show.month, data2show.day, data2show.hour, data2show.minute);
	u8g.drawStr(6, 50, buffer);
	
}

void uiSetTimeCursor()
{
	u8g.drawRFrame(4+cursor_cambio_hora*24, 40, 15, 13,2);
}

void uiSetTime(struct data* data2change)
{
	ajustar_hora(data2change);
	u8g.setFont(u8g_font_6x10);
	u8g.drawStr(6, 10, "Ajustar hora");
	u8g.drawLine(6, 12, 120, 12); //Seperator
	u8g.drawStr(6, 30, "AÑO MES DIA HOR MIN");
	uiSetTimeShowTime(*data2change);
	uiSetTimeCursor();
}


/*  ----------------------------------------------------------------------------------------
 *  uiProg functions
 *  ----------------------------------------------------------------------------------------
 */



void uiProgShowData(struct data data2prog)
{
	char buffer[30];
	sprintf(buffer, "%02d:%02d %02d:%02d %02d:%02d", data2prog.prog_1[0], data2prog.prog_1[1], data2prog.prog_2[0], data2prog.prog_2[1], data2prog.prog_3[0], data2prog.prog_3[1]);
	u8g.drawStr(25, 40, buffer);
	sprintf(buffer, "%02d:%02d %02d:%02d %02d:%02d", data2prog.prog_1[2], data2prog.prog_1[3], data2prog.prog_2[2], data2prog.prog_2[3], data2prog.prog_3[2], data2prog.prog_3[3]);
	u8g.drawStr(25, 60, buffer);
}

void uiProgCursor_Seleccion()
{
	u8g.drawRFrame(23+cursor_prog_seleccion*36, 30, 33, 33,2);
}

void uiProgCursor_Edicion()
{
	if (cursor_prog_arriba)
	{
		u8g.drawRFrame(cursor_prog_edicion*18+23+cursor_prog_seleccion*36, 30, 15, 13,2);
	}else
	{
		u8g.drawRFrame(cursor_prog_edicion*18+23+cursor_prog_seleccion*36, 50, 15, 13,2);
	}
	
}

void uiProg(struct data* data2prog)
{
	u8g.setFont(u8g_font_6x10);
	u8g.drawStr(4, 10, "Programacion diaria");
	u8g.drawLine(4, 12, 120, 12); //Seperator
	u8g.drawStr(4, 25, "MEM   1     2     3");
	u8g.drawStr(4, 40, "INI");
	u8g.drawStr(4, 60, "FIN");
	
	
	if (cursor_prog_abajo | cursor_prog_arriba)
	{
		uiProgCursor_Edicion();
	}else
	{
		uiProgCursor_Seleccion();
	}
	ajustar_prog(data2prog);
	uiProgShowData(*data2prog);

}

/*  ----------------------------------------------------------------------------------------
 *  Temp-related functions
 *  ----------------------------------------------------------------------------------------
 */

void calcular_temp(unsigned char* cambios, int* temperatura)
{
	int8_t i;
	uint8_t f;
	rtc_force_temp_conversion(1);
	ds3231_get_temp_int(&i, &f);
	*temperatura=i*100+f;
	*cambios|=(1<<TEMP);
}


void comprobar_max(int temp_amb)
{
	
	if (temp_amb > temp_max)
	{
		temp_max = temp_amb;
	}
	
	if ((temp_amb < temp_min) && (temp_amb>100))
	{
		temp_min=temp_amb;
	}
}

void comprobar_temp(struct data* data2draw)
{
	if ((data2draw->estado==MANUAL) | (data2draw->estado==PROGRAMADO_ON))
	{
		if (data2draw->temperatura+MARGEN_TEMP<data2draw->temp_prog)
		{
			data2draw->activado=1;
		}else if (data2draw->temperatura-MARGEN_TEMP>data2draw->temp_prog)
		{
			data2draw->activado=0;
		}
	}else
	{
		data2draw->activado=0;
	}
	if (data2draw->activado)
	{
		activar_radiador();
	}
	
}

void calcular_temp_media(struct data data2process)
{
	long suma_temperaturas=0;
	int x;
	
	if (cursor_temperaturas==NUM_MAX_MUESTRAS) cursor_temperaturas=0;
	temperaturas[cursor_temperaturas]=data2process.temperatura;
	
	if (temperaturas[NUM_MAX_MUESTRAS-1]>1) x=NUM_MAX_MUESTRAS-1;
	else x=cursor_temperaturas;
	
	for (int y=x;y>=0;y--)
	{
		suma_temperaturas=suma_temperaturas+temperaturas[y];
	}
	temp_media=(suma_temperaturas/(x+1))/100;
	cursor_temperaturas++;
}


/*  ----------------------------------------------------------------------------------------
 *  Other functions
 *  ----------------------------------------------------------------------------------------
 */

void ajustar_hora(struct data* data2change)
{
	 
	if (flag_subir_hora)
	{
		switch (cursor_cambio_hora) {
			case 0:
			if (data2change->year>=30) data2change->year=14;
			else data2change->year++;
			break;
			case 1:
			if (data2change->month>=12) data2change->month=1;
			else data2change->month++;
			break;
			case 2:
			if (data2change->day>=31) data2change->day=1;
			else data2change->day++;
			break;
			case 3:
			if (data2change->hour>=23) data2change->hour=00;
			else data2change->hour++;
			break;
			case 4:
			if (data2change->minute>=59) data2change->minute=00;
			else data2change->minute++;
			break;
			default:
			data2change->minute++;
		}
		rtc_set_time(data2change->minute, data2change->hour, 1, data2change->day, data2change->month, data2change->year);
		flag_subir_hora=0;
	}else if (flag_bajar_hora)
	{
		switch (cursor_cambio_hora) {
			case 0:
			if (data2change->year<=14) data2change->year=30;
			else data2change->year--;
			break;
			case 1:
			if (data2change->month<=1) data2change->month=12;
			else data2change->month--;
			break;
			case 2:
			if (data2change->day<=1) data2change->day=31;
			else data2change->day--;
			break;
			case 3:
			if (data2change->hour<=0) data2change->hour=23;
			else data2change->hour--;
			break;
			case 4:
			if (data2change->minute<=0) data2change->minute=59;
			else data2change->minute--;
			break;
			default:
			data2change->minute--;
		}
		rtc_set_time(data2change->minute, data2change->hour, 1, data2change->day, data2change->month, data2change->year);
		flag_bajar_hora=0;
	}
}


void editar_programas(unsigned char programa[NUM_PROGS+1],int posicion, unsigned char* cambios)
{
	
	if (cursor_prog_edicion)
	{
		if (flag_subir_prog)
		{
			if (programa[1+posicion]<45)
			{
				programa[1+posicion]+=INCREMENTO_PROG;
			}else
			{
				programa[1+posicion]=0;
			}
			flag_subir_prog=0;
		}else if (flag_bajar_prog)
		{
			if (programa[1+posicion]>14)
			{
				programa[1+posicion]-=INCREMENTO_PROG;
			}else
			{
				programa[1+posicion]=45;
			}
			flag_bajar_prog=0;
		}
	}else
	{
		if (flag_subir_prog)
		{	
			if (programa[posicion]<23)
			{
				programa[posicion]++;
			}else
			{
				programa[posicion]=0;
			}
			flag_subir_prog=0;
		}else if (flag_bajar_prog)
		{		
			if (programa[posicion]>0)
			{
				programa[posicion]--;
			}else
			{
				programa[posicion]=23;
			}
			flag_bajar_prog=0;
		}
	}
	*cambios|=(1<<PROG);
}

void ajustar_prog(struct data* data2change)
{
	switch (cursor_prog_seleccion) {
		
		case 0:
			if (cursor_prog_arriba)
			{
				editar_programas(data2change->prog_1, 0, &data2change->cambios);
				
			}else if (cursor_prog_abajo)
			{
				editar_programas(data2change->prog_1, 2, &data2change->cambios);
			}
		break;
		case 1:
			if (cursor_prog_arriba)
			{
				editar_programas(data2change->prog_2, 0, &data2change->cambios);
			}else if (cursor_prog_abajo)
			{
				editar_programas(data2change->prog_2, 2, &data2change->cambios);
			}
		break;
		case 2:
			if (cursor_prog_arriba)
			{
				editar_programas(data2change->prog_3, 0, &data2change->cambios);
			}else if (cursor_prog_abajo)
			{
				editar_programas(data2change->prog_3, 2, &data2change->cambios);
			}
		break;
		default:
		if (cursor_prog_arriba)
		{
			editar_programas(data2change->prog_1, 0, &data2change->cambios);
		}else if (cursor_prog_abajo)
		{
			editar_programas(data2change->prog_1, 2, &data2change->cambios);
		}
	}
}


void draw(struct data* data2draw)
{
	switch (menu_seleccionado) {
		case MENU_PPL:
			uiMain(*data2draw);
		break;
		case PROGRAMACION:
			uiProg(data2draw);
		break;
		case CAMBIAR_HORA:
			uiSetTime(data2draw);
		break;
		case ESTADISTICAS:
			uiStats();
		break;
		default:
			uiMain(*data2draw);
	}
}

void activar_radiador()
{

}

void botones(struct data* data2changue)
{

	
	if (flag_subir | contador_subir_larga)
	{
				
		if (menu_seleccionado==MENU_PPL)
		{
			if (bit_is_clear(pinBotones,pinSubir)){
				contador_subir_larga++;
				if (contador_subir_larga==PRESCALER_PUL_LARGA){
					selected_station < NUM_STATIONS-1 ? selected_station++ : selected_station=0;
					contador_subir_larga=0;		
				}
			}else{
					data2changue->temp_prog+=100;
					data2changue->cambios|=(1<<TEMP);
					flag_actualizar_radiador=1;
					contador_subir_larga=0;
			}
			
			
		}else if (menu_seleccionado==PROGRAMACION)
		{
			if (cursor_prog_abajo | cursor_prog_arriba)
			{
				flag_subir_prog=1;
			}else
			{
				cursor_prog_arriba=1;
			}
		}else if (menu_seleccionado==CAMBIAR_HORA)
		{
			flag_subir_hora=1;
		}
		
		flag_subir=0;
		
	}else if (flag_bajar | contador_bajar_larga)
	{
		if (menu_seleccionado==MENU_PPL)
		{
			if (bit_is_clear(pinBotones,pinBajar)){
				contador_bajar_larga++;
				if (contador_bajar_larga==PRESCALER_PUL_LARGA){
					selected_station > 0 ? selected_station-- : selected_station=NUM_STATIONS-1;	
					contador_bajar_larga=0;	
				}
			}else{
					data2changue->temp_prog-=100;
					contador_bajar_larga=0;
					data2changue->cambios|=(1<<TEMP);
					flag_actualizar_radiador=1;
			}
			
		}else if (menu_seleccionado==PROGRAMACION)
		{
			if (cursor_prog_abajo | cursor_prog_arriba)
			{
				flag_bajar_prog=1;
			}else
			{
				cursor_prog_abajo=1;
			}
		}else if (menu_seleccionado==CAMBIAR_HORA)
		{
			flag_bajar_hora=1;
		}
		flag_bajar=0;
		
	}else if (flag_izqd)
	{
		if (menu_seleccionado==CAMBIAR_HORA)
		{
			if (cursor_cambio_hora==0)
			{
				menu_seleccionado=CAMBIAR_HORA-1;
				cursor_prog_seleccion=NUM_PROGS;
			}else
			{
				cursor_cambio_hora--;
			}
		}else if (menu_seleccionado==PROGRAMACION)
		{
			if (!cursor_prog_abajo & !cursor_prog_arriba)
			{
				if (cursor_prog_seleccion==0)
				{
					menu_seleccionado=PROGRAMACION-1;
				}else
				{
					cursor_prog_seleccion--;
				}
			}else
			{
				cursor_prog_edicion=!cursor_prog_edicion;
			}
		}else if (menu_seleccionado==0)
		{
			menu_seleccionado=NUM_MENUS;
			cursor_cambio_hora=NUM_CURSOR_HORA;
		
		}else
		{
			menu_seleccionado--;
		}
		flag_izqd=0;
	
	}else if (flag_drcha)
	{
		if (menu_seleccionado==CAMBIAR_HORA)
		{
			if (cursor_cambio_hora==NUM_CURSOR_HORA)
			{
				menu_seleccionado=0;
				cursor_cambio_hora=0;
			}else
			{
				cursor_cambio_hora++;
			}
		}else if (menu_seleccionado==PROGRAMACION)
		{
			if (!cursor_prog_abajo & !cursor_prog_arriba)
			{
				if (cursor_prog_seleccion==NUM_PROGS)
				{
					menu_seleccionado=PROGRAMACION+1;
					cursor_prog_seleccion=0;
				}else
				{
					cursor_prog_seleccion++;
				}
			}else
			{
				cursor_prog_edicion=!cursor_prog_edicion;
			}
		}else if (menu_seleccionado==NUM_MENUS)
		{
			menu_seleccionado=0;
		
		}else
		{
			menu_seleccionado++;
		}
		flag_drcha=0;
	
	}else if (flag_ok)
	{
		if (menu_seleccionado==PROGRAMACION)
		{
			if ((cursor_prog_abajo==0) & (cursor_prog_arriba==0) & (cursor_prog_edicion==0))
			{
				cursor_prog_arriba=1;
			}else{
				cursor_prog_abajo=0;
				cursor_prog_arriba=0;
				cursor_prog_edicion=0;
			}
		}else if(menu_seleccionado==MENU_PPL)
		{
			if (data2changue->estado==MANUAL)
			{
				data2changue->estado=PROGRAMADO_OFF;
			}else if((data2changue->estado==PROGRAMADO_OFF) | (data2changue->estado==PROGRAMADO_ON))
			{
				data2changue->estado=MANUAL;
			}
			flag_actualizar_radiador=1;
			
				
		}
		flag_ok=0;
	}
	
	if (bit_is_set(PINONOFF,pinONOFF))
	{
		if (!data2changue->estado)
		{
			flag_actualizar_radiador=1;
			data2changue->estado=MANUAL;
			
		}
		
		
	}else if (!bit_is_set(PINONOFF,pinONOFF))
	{
		if (data2changue->estado)
		{
			flag_actualizar_radiador=1;
			data2changue->estado=OFF;
			
		}
	}

}

void comprobar_programacion(struct data data2check)
{
	if ((data2check.estado==PROGRAMADO_ON) | (data2check.estado==PROGRAMADO_OFF))
	{
		unsigned char* programas[]={data2check.prog_1, data2check.prog_2, data2check.prog_3};
	
		for (int x=0;x<=NUM_PROGS;x++)
		{
			if ((data2check.hour==programas[x][0]) & (data2check.minute==programas[x][1]))
			{
				data2check.estado=PROGRAMADO_ON;
				
			}else if ((data2check.hour==programas[x][2]) & (data2check.minute==programas[x][3]))
			{
				data2check.estado=PROGRAMADO_OFF;
				
			}
		}
	}
}


void sleepNow()
{
	// // Choose our preferred sleep mode:
	// set_sleep_mode(SLEEP_MODE_IDLE);
	
	// // Set sleep enable (SE) bit:
	// sleep_enable();
	
	// // Put the device to sleep:
	// sleep_mode();
	
	// // Upon waking up, sketch continues from this point.
	// sleep_disable();
	
}

/*  ----------------------------------------------------------------------------------------
 *  Main
 *  ----------------------------------------------------------------------------------------
 */

void setup(){}

void loop(void)
{
	//LCD init
	u8g.setHardwareBackup(u8g_backup_avr_spi);
	u8g.setRot180();
	Serial.begin(57600);
	printf_begin();
	set_up_timer();
	set_up_io();
	set_up_nRF24L01();
	Wire.begin();
	sei();
	calcular_temp(&stations[STATION].cambios, &stations[STATION].temp_anterior);
	
	
	int x=0;
	
  for(;;)
  {  
	  
	  if (!contador_actualizar_botones--)
	  {
		  contador_actualizar_botones=PRESCALER_BOTONES;
		  botones(&stations[selected_station]);
	  }
	
	 if (flag_encender_pantalla)
	 {
		portLcdBacklight |= (1<<pinLcdBacklight);
		
		//sleepNow();
		//portLcdBacklight &= ~(1<<pinLcdBacklight);
	 }else{
		portLcdBacklight &= ~(1<<pinLcdBacklight);
	 }
	  
	 if (flag_actualizar_tiempo)
	 {
		 rtc_get_time(stations);
		 comprobar_programacion(stations[STATION]);
		 flag_actualizar_tiempo=0;
	 }
	 
	 if (flag_actualizar_temp){
		stations[STATION].temp_anterior=stations[STATION].temperatura;
		calcular_temp(&stations[STATION].cambios, &stations[STATION].temperatura);
		comprobar_max(stations[STATION].temperatura);
		flag_actualizar_radiador=1;
		flag_actualizar_temp=0;
	 }
	 
	 if (flag_actualizar_radiador)
	 {
		
		 comprobar_temp(&stations[STATION]);
		 calcular_temp_media(stations[STATION]);
		 flag_actualizar_radiador=0;
	 }	  
	 
	 if (flag_envio)
	 {
		 Send_Data(1, stations);
	 }

	if (x++==500)
	{
		x=0;
		u8g.firstPage();
		do
		{
			draw(&stations[selected_station]);
		} while ( u8g.nextPage() );}
  } 
}


/*  ----------------------------------------------------------------------------------------
 *  Interrupts
 *  ----------------------------------------------------------------------------------------
 */

ISR(TIMER1_COMPA_vect)
{
	if (!contador_actualizar_tiempo--)
	{
		contador_actualizar_tiempo=PRESCALER_TIEMPO;
		flag_actualizar_tiempo=1;
	}
	
	if (!contador_actualizar_temp--)
	{
		contador_actualizar_temp=PRESCALER_TEMP;
		flag_actualizar_temp=1;
	}
	
	if (!contador_actualizar_radiador--)
	{
		contador_actualizar_radiador=PRESCALER_RADIADOR;
		//flag_actualizar_radiador=1;
	}	
	
	if (!contador_encender_pantalla--)
	{
		contador_encender_pantalla=PRESCALER_LCD;
		flag_encender_pantalla=1;
	}
	if (!contador_envio--)
	{
		contador_envio=PRESCALER_ENVIO;
		flag_envio=1;
	}
}

ISR(PCINT2_vect)
{
	if(!flag_encender_pantalla)
	{
		if (!bit_is_set(pinBotones,pinSubir))
		{
			flag_subir=1;
		}else if (!bit_is_set(pinBotones,pinBajar))
		{
			flag_bajar=1;
		}else if (!bit_is_set(pinBotones,pinIzqd))
		{
			flag_izqd=1;
		}else if (!bit_is_set(pinBotones,pinDrcha))
		{
			flag_drcha=1;
		}else if (!bit_is_set(pinBotones,pinOk))
		{
			flag_ok=1;
		}
		contador_encender_pantalla=PRESCALER_LCD;
		
	}else{
		flag_encender_pantalla=0;
	}
}