

#ifndef _mcp2515_H_
#define _mcp2515_H_



#include "SPI_0_AVR128DA64.h"
#include "UART_1_AVR128DA64.h"




#define CS_PORT	PORTA
#define CS_PIN	PIN7_bm
#define rcv_can	1
#define snd_can 1
#define max_valve_open 6
unsigned long millis,millis_2,sec_t;

#define CAN_EFF_FLAG 0x80000000UL	/* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000UL	/* remote transmission request */
#define CAN_ERR_FLAG 0x20000000UL	/* error message frame */

#define CAN_SFF_ID_BITS     11
#define CAN_EFF_ID_BITS     29

#define CAN_MAX_DLC 8
#define CAN_MAX_DLEN 8

#define CAN_SFF_MASK 0x000007FFUL	/* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFUL	/* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFUL	/* omit EFF, RTR, ERR flags */
char G;
struct can_frame {
	
	unsigned long   can_id;		/* 32 bit CAN_ID + EFF/RTR/ERR flags */
	unsigned char    can_dlc;	/* frame payload length in byte (0 .. CAN_MAX_DLEN) */
	unsigned char    data[8];
	
};

struct can_frame canMsg1;

uint8_t txBuffers[3] = {0, 1, 2};
int set_prssr = 20;
uint8_t etcuff_mode=1;
unsigned long sendTime=0;
int rcv_can_stus=0 , snd_can_status=0;
unsigned char data_prssre = 0;
unsigned int can_rcv_vle=0;
char send[8]="";
char buff_prssr1[10];
char buff_prssr2[10];
//float current_pressure =0;
int prssr_vle_hgh = 25;
int prssr_vle_lw = 20;
bool extubate_flag = true;
int value_open_count = max_valve_open;
/**************************************** Function Declarations ******************************************************/

void set_millis_2(void);
void etcuff_send(float send_prssre);
void can_send(float send_prssre);
void spi_init_master(void);
uint8_t spi_tranceiver( uint8_t d);
void mcp_set_register(const uint8_t add, const uint8_t value);
void mcp_set_registers(const uint8_t add, const uint8_t values[], const uint8_t n);
void mcp_modify_register(const uint8_t add, const uint8_t mask, const uint8_t data);
void prepareId(uint8_t *buffer, const bool ext, const uint32_t id);
uint8_t setFilterMask(const uint8_t mask, const bool ext, const uint32_t ulData);
uint8_t setConfigMode();
uint8_t mcp_read_register(const uint8_t add);
void mcp_read_registers(const uint8_t add , uint8_t values[], const uint8_t n);
uint8_t setMode(const uint8_t mode);
uint8_t setFilter(const uint8_t num, const bool ext, const uint32_t ulData);
uint8_t reset(void);
uint8_t sendMessagek(const uint8_t txbn, const struct can_frame *frame);
uint8_t sendMessage(const struct can_frame *frame);
void mcp_init(void);
uint8_t readmsg(struct can_frame *frame);
uint8_t readmsgk(const uint8_t rxbn, struct can_frame *frame);
uint8_t read_status();
void end_spi(void);
void start_spi(void);
void send_data(unsigned long id, unsigned char l_dlc,char *str);
void rcv_data();
void strset(char *str);
void External_interrrupt_init(void);
uint8_t data_filter_mode(const unsigned char data);
void etcuff_PT_STATUS_send(int ptint_sts);
int etcuff_rcv(void);

/********************************* Millis for Status send every 1 sec ******************************************/



void set_millis_2(void)		
{
	sei();
	TCB1_CCMP = 11999;				// Write a TOP value to the Compare/Capture (TCBn.CCMP) register
	
	TCB1_CTRLB |= (0x0 << 0);
	TCB1_INTCTRL |= (1<<0);
	
	TCB1_CTRLA |= (1<<0)|(0x1 <<1); // ENABLE bit in the Control A (TCBn.CTRLA) register,

}


ISR(TCB1_INT_vect)
{
	millis_2++;
	if(millis_2 >1000)
	{

		snd_can_status = 1;
		sec_t=sec_t+1;
		millis_2 = 0 ;
	}
	
	TCB1_INTFLAGS |= (1<<0);
}


/****************************************  Function to send and rcv Pressure status on can bus  ****************************************************/

void can_send(float send_prssre)
{
	if(snd_can_status == snd_can)
	{
		
		
		
		strset(send);
 		intToStr(send_prssre,buff_prssr2,1);
 		send[0] = buff_prssr2[0];
 		send[1] = buff_prssr2[1];
 		//send[2] = 0x2C;
 		
		
		//intToStr(etcuff_mode,buff_prssr1,1);
		//send[3] = buff_prssr1[0];
		
		//send[4] = 0x2C;
		//
		//send[5] = buff_prssr2[0];
		//send[6] = buff_prssr2[1];
		
		send_data(0x07,2,send);
		
	//	USART1_sendString(send);
		snd_can_status = 0;
		
		_delay_ms(100);
		
		
		
	}


}

int etcuff_rcv(void)
{	
	if(rcv_can_stus == rcv_can)
	{
		
		PORTE_OUT &= ~(1 << 0);					// SOL 1 OFF
		 PORTE_OUT &= ~(1 << 1);					// SOL 2 OFF
		 TCA0.SINGLE.CTRLA |= (1 << 0);			//MOTOR PWM enable
		 TCA0.SINGLE.CMP0 = 0;					//dutyCycle
		
		readmsg(&canMsg1);
		_delay_ms(5);
		
		if(canMsg1.can_id == 0x07)
		{
			
		
		etcuff_mode = data_filter_mode(canMsg1.data[4]);
		
		
		data_prssre = (canMsg1.data[6]-48)*10 + (canMsg1.data[7]-48);
		
		
		set_prssr = data_prssre;
		if(etcuff_mode == 4)
		{
			extubate_flag = true;
		}
		else
		{
			extubate_flag = false;
		}
		}
		

		value_open_count = max_valve_open;
		rcv_can_stus = 0;
		
		_delay_ms(10);
		
		return data_prssre;
	}
	else
	{
	
		return set_prssr ;
	}
}


/***************************************  Patient  status send function   ******************************************************/

void etcuff_PT_STATUS_send(int ptint_sts)
{
	
	strset(send);
	
	if(ptint_sts==0)				//G@,ACK01  PDC
	{
		
	send[0] = 0x47;
	send[1] = 0x40;
	send[2] = 0x41;
	send[3] = 0x43;
	send[4] = 0x4B;
	send[5] = 0x30;
	send[6] = 0x31;
	send_data(0x02,7,send);
	
	
	}
	else if(ptint_sts==1)			//G@,ACK00  PC
	{
		
		send[0] = 0x47;
		send[1] = 0x40;
		send[2] = 0x4E;
		send[3] = 0x41;
		send[4] = 0x43;
		send[5] = 0x4B;
		send[6] = 0x30;
		send[7] = 0x30;
		send_data(0x02,8,send);
		
	}
	else if(ptint_sts==2)			//G@,ACK11 NO LEAKAGE
	{
		
		send[0] = 0x47;
		send[1] = 0x40;
		send[2] = 0x4E;
		send[3] = 0x41;
		send[4] = 0x43;
		send[5] = 0x4B;
		send[6] = 0x31;
		send[7] = 0x30;
		send_data(0x02,8,send);
		
	}
	else if(ptint_sts==3)			//G@,ACK11 LEAKAGE
	{
		
		send[0] = 0x47;
		send[1] = 0x40;
		send[2] = 0x41;
		send[3] = 0x43;
		send[4] = 0x4B;
		send[5] = 0x31;
		send[6] = 0x31;
		send_data(0x02,7,send);
	}
	
	
	
	USART1_sendString("send");
	
	
	_delay_ms(10);
}

/************************************* Interrupt to read data from can bus *************************************/

void External_interrrupt_init(void)
{
	sei();
	PORTD.PIN2CTRL = (0x3<< 0)|(1 << 3); //
	
}


ISR(PORTD_PORT_vect)
{
	if (PORTD.INTFLAGS & (1 << 2))
	{
		
		rcv_can_stus = 1;
		
		PORTD.INTFLAGS |= (1 << 2);
	}
	
}



/*********************************  MCP_2515 Initialization ***************************************************************/

void mcp_init(void)
{
	start_spi();
	PORTC.DIRSET = CS_PIN;
	PORTC_OUT |=(1<<6);
	end_spi();
	
	
	
	reset();									//RESET FUNCTION RESET ALL REGISTER ON MCP2515 TO INITIAL VALUE
	
	setConfigMode();							// SRT IN CONFIG MODE TO SET SPEED OF CAN TO 125KBPs
	uint8_t  cfg1, cfg2, cfg3;					
												/////////SPEED 125KBPs
	cfg1 = 0x03;		
	cfg2 = 0xF0;		
	cfg3 = 0x86;		
	
	mcp_set_register(0x2A, cfg1);				// SET CNF1 REGISTER
	mcp_set_register(0x29, cfg2);				// SET CNF2 REGISTER
	mcp_set_register(0x28, cfg3);				// SET CNF3 REGISTER
	
	setMode(0x00);								//SET NORMAL MODE
	
	External_interrrupt_init();					//INITIALIZE EXTERNAL INTERRUPT
	
}



/***************************************** SPI SEND AND RECEIVE FUNCTION ************************************/

uint8_t spi_tranceiver(uint8_t data)
{
	SPI_0_send_char(data);
}



/**************************************  fUNCTION TO SET REGISTER OF MCP2515 *******************************************************/

void mcp_set_register(const uint8_t add, const uint8_t value)
{
	start_spi();
	
	spi_tranceiver(0x02);		//0x02 --> instruction write
	spi_tranceiver(add);		//add --> address of register
	spi_tranceiver(value);
	
	end_spi();
	
	_delay_ms(10);
}

/*************************************** fUNCTION TO SET REGISTERS OF MCP2515 ****************************************************/


void mcp_set_registers(const uint8_t add, const uint8_t values[], const uint8_t n)
{
	
	start_spi();								//spi enable
	
	spi_tranceiver(0x02);						//0x02 --> instruction write
	spi_tranceiver(add);						//add --> address of register to begin modify 
	for (uint8_t i = 0; i < n; i++)
	{
		spi_tranceiver(values[i]);
	}
	
	end_spi();									//spi disable
	
	_delay_ms(10);
}


/*********************************************  fUNCTION TO modify bits of REGISTER   *******************************************************************/


void mcp_modify_register(const uint8_t add, const uint8_t mask, const uint8_t data)
{
	start_spi();		//spi enable
	
	spi_tranceiver(0x05);			//to  modify any bit of registers use 0x05
	spi_tranceiver(add);
	spi_tranceiver(mask);			//mask is required as only few registers can be bit manipulated
	spi_tranceiver(data);
	
	end_spi();		//spi disable
	
	_delay_ms(10);
}


/********************************************** Function to create id from id registers ***********************************************/

void prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
	uint16_t canid = (uint16_t)(id & 0x0FFFF);

	if (ext)
	{
		buffer[3] = (uint8_t) (canid & 0xFF);
		buffer[2] = (uint8_t) (canid >> 8);
		canid = (uint16_t)(id >> 16);
		buffer[1] = (uint8_t) (canid & 0x03);
		buffer[1] += (uint8_t) ((canid & 0x1C) << 3);
		buffer[1] |= 0x08;
		buffer[0] = (uint8_t) (canid >> 5);
	}
	else
	{
		buffer[0] = (uint8_t) (canid >> 3);					// TXBnSIDH transmit buffer have bit[10:3] so 11bit SID make shift by 3
		buffer[1] = (uint8_t) ((canid & 0x07 ) << 5);		// TXBnSIDL TANSMIT BUFFER have bit[7-5]
		//as SID so only 3 bit so and 1 byte can have max 8 bit so &07 and <<5 shift is required.
		buffer[3] = 0;
		buffer[2] = 0;
	}
}


/***************************************  Function USED TO UPDATE ALL MASK REGISTERS    **************************************************/


uint8_t setFilterMask(const uint8_t mask, const bool ext, const uint32_t ulData)
{
	uint8_t res = setConfigMode();
	if (res != 0) {
		return res;
	}
	
	uint8_t tbufdata[4];
	prepareId(tbufdata, ext, ulData);

	uint8_t reg;
	switch (mask) {
		case 0: reg = 0x20; break;
		case 1: reg = 0x24; break;
		default:
		return 1;
	}

	mcp_set_registers(reg, tbufdata, 4);
	
	return 0;
}

/******************************************   Set mcp2515 in configuration mode  ****************************************************************/

uint8_t setConfigMode()
{
	return setMode(0x80);				//0x80 is config mode
}

/*****************************************  Read resgiters      ***************************************************/

uint8_t read_status()
{
	start_spi();
	
	spi_tranceiver(0xA0);		//RX READ STATUS 0xA0
	uint8_t i = spi_tranceiver(0x00);
	
	
	end_spi();
	
	_delay_ms(10);
	return i;
	
}


/********************************************* Function to "Read register" of mcp2515   *******************************************************/

uint8_t mcp_read_register(const uint8_t add)
{
	uint8_t ret;
	start_spi();
	
	spi_tranceiver(0x03);			//0x03 read spi cmd
	spi_tranceiver(add);
	ret = spi_tranceiver(0x00);		//In read mode spi will recieve data when transfer 0x00
	
	end_spi();
	
	return ret;
	
}



/****************************************** Function to "Read registers" of mcp2515 *****************************************************************/

void mcp_read_registers(const uint8_t add , uint8_t values[], const uint8_t n)
{
	
	start_spi();
	
	spi_tranceiver(0x03);			//0x03 read mode SPI
	spi_tranceiver(add);			//ADD of resgister to read
	for (uint8_t i=0; i<n; i++)
	{
		values[i] = spi_tranceiver(0x00);		//data will be sent to consecutive register
	}
	
	end_spi();
	
}

/********************************************* Function to Read message for particular buffer of mcp2515   *******************************************************/


uint8_t readmsgk(const uint8_t rxbn, struct can_frame *frame)
{
	const uint8_t rxb_ctrl[2] = {0x60,0x70};			//RX CONTROL REGISTERS
	const uint8_t rxb_sidh[2] = {0x61,0x71};			//RX IDh REGISTERS
	const uint8_t rxb_sidl[2] = {0x62,0x72};			//RX IDl REGISTERS
	const uint8_t rxb_EID8[2] = {0x63,0x73};			//RX EID8 REGISTERS
	const uint8_t rxb_EID0[2] = {0x64,0x74};			//RX EID0 REGISTERS
	const uint8_t rxb_dlc[2] = {0x65,0x75};				//RX DLC REGISTERS
	const uint8_t rxb_data[2] = {0x66,0x76};			//RX DATA REGISTERS
	const uint8_t CANINTF_RXnIF[2] = {0x01,0x02};			//RX DATA REGISTERS
	
	
	uint8_t tbufdata[5];
	
	mcp_read_registers(rxb_sidh[rxbn], tbufdata, 5);	// read all registers till DLC
	
	uint32_t id = (tbufdata[0]<<3)+(tbufdata[1]>>5);	//bit shifting to get id
	
	if ( (tbufdata[1] & 0x08) ==  0x08 ) {				//if EXIDE bit in TXB MCP_SIDL IS 1 then exide is used
		id = (id<<2) + (tbufdata[1] & 0x03);
		id = (id<<8) + tbufdata[2];				//EID8
		id = (id<<8) + tbufdata[3];				//EID0
		id |= CAN_EFF_FLAG;
	}
	
	uint8_t dlc = tbufdata[4] & 0x0F;				//tbufdata 4th element is dlc register value
	
	if(dlc>8)										//if dlc length is >8 then error
	{
		return 1;
	}
	
	uint8_t ctrl = mcp_read_register(rxb_ctrl[rxbn]);		//read rx control register and
	
	if (ctrl & 0x08) {									//RXBnCTRL_RTR
		id |= CAN_RTR_FLAG;								// remote request received
	}
	
	frame->can_id = id;
	frame->can_dlc = dlc;
	
	mcp_read_registers(rxb_data[rxbn], frame->data, dlc);		//now read data registers and get data bytes
	
	mcp_modify_register(0x2C, CANINTF_RXnIF[rxbn], 0 );			// after receive done then clear CANINTF FLAG to 0.
	
	return 0;
	
	

}


/**********************************************  Used for message read In main function*******************************************/


uint8_t readmsg(struct can_frame *frame)
{
	uint8_t rxs;
	uint8_t stat = read_status();
	
	if(stat & (1<<0))
	{
		rxs = readmsgk(0,frame);		//USE RECEIVE BUFFER 0
	}
	else if(stat & (1<<1))
	{
		rxs = readmsgk(1,frame);		//USE RECEIVE BUFFER 1
	}
	else
	{
		rxs = 2;						// no msg receive flag
	}
	return rxs;
}



/**********************************************  Used for Mode set of mcp2515*******************************************/



uint8_t setMode(const uint8_t mode)
{
	mcp_modify_register(0x0F, 0xE0, mode);			// 0x0F --> address of CANCTRL register
	// 0XE0 --> BITs need to modify i.e. bit[7-5] of CANCTRL byte
	// mode --> mode required.
	
	unsigned long endTime = millis + 10;
	bool modeMatch = false;
	while (millis < endTime) {
		uint8_t newmode = mcp_read_register(0x0E);			//read CANSTAT register(0x0E) for mode
		newmode &= 0xE0;

		if (newmode == mode)		//MATCH NODE
		{
			break;
		}
	}

	return modeMatch ? 0 : 1;

}

/********************************************** USED TO UPDATE ALL FILTER REGISTERS*******************************************/


uint8_t setFilter(const uint8_t num, const bool ext, const uint32_t ulData)
{
	uint8_t res = setConfigMode();
	if (res != 0) {
		return res;
	}

	uint8_t reg;

	switch (num) {
		case 0: reg = 0x00; break;
		case 1: reg = 0x04; break;
		case 2: reg = 0x08; break;
		case 3: reg = 0x10; break;
		case 4: reg = 0x14; break;
		case 5: reg = 0x18; break;
		default:
		return 1;
	}

	uint8_t tbufdata[4];
	prepareId(tbufdata, ext, ulData);
	mcp_set_registers(reg, tbufdata, 4);

	return 0;
}


/********************************************** Start and end spi transmission *******************************************/

void start_spi(void)
{
	SPI_0_init();
	CS_PORT.OUTCLR = CS_PIN;
	
}

void end_spi(void)
{
	CS_PORT.OUTSET = CS_PIN;
	SPI0.CTRLA &=~ (1 << 0);
}

/********************************************* Reset mcp2515 to initial state  ******************************************/

uint8_t reset(void)
{
	start_spi();
	
	spi_tranceiver(0xC0);
	
	end_spi();
	_delay_ms(10);
	
	uint8_t zeros[14];
	memset(zeros, 0, sizeof(zeros));
	mcp_set_registers(0x30, zeros, 14);		//since initializing so make every bit clear thatswhy zeros
	mcp_set_registers(0x40, zeros, 14);
	mcp_set_registers(0x50, zeros, 14);
	
	mcp_set_register(0x60, 0);
	mcp_set_register(0x70, 0);
	
	mcp_set_register(0x2B, 0x01 | 0x02 | 0x20 | 0x80);
	
	
	mcp_modify_register(0x60,0x60 | 0x04 | 0x03,0x00 | 0x04 | 0x00);
	//mcp_modify_register(0x60,0x48,0x48);
	
	
	mcp_modify_register(0x70,0x60 | 0x07,0x00 | 0x01);
	///mcp_modify_register(0x70,0x48,0x48);
	
	uint8_t filters[] = {0, 1, 2, 3, 4, 5};
	
	
	for (int i=0; i<6; i++) {
		bool ext = 0;
		uint8_t result = setFilter(filters[i], ext, 0);
		if (result != 0) {
			return result;
		}
	}


	uint8_t masks[] = {0, 1};
	for (int i=0; i<2; i++) {
		uint8_t result = setFilterMask(masks[i], true, 0);
		if (result != 0) {
			return result;
		}
	}

	return 0;

	
	
}


/********************************************** USED TO Send message according to buffer *******************************************/


uint8_t sendMessagek(const uint8_t txbn, const struct can_frame *frame)
{
	if (frame->can_dlc > 8) {
		return 4;
	}
	
	//const struct TXBn_REGS *txbuf = &TXB[txbn];
	
	uint8_t data[13];
	
	uint8_t tx_ctrl[3] = {0x30,0x40,0x50};				//TRANSMIT CONTROL REGISTERS
	uint8_t tx_sidh[3] = {0x31,0x41,0x51};				//TRANSMIT ID REGISTERS
	
	bool ext = (frame->can_id & CAN_EFF_FLAG);			//CHECK EXTENDED ID IS USED
	bool rtr = (frame->can_id & CAN_RTR_FLAG);			//CHECK RTR
	uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

	prepareId(data, ext, id);				//PREPARE ID I.E. SAVE DATA BUFFER WITH ID VALUES

	data[4] = rtr ? (frame->can_dlc | 0x40) : frame->can_dlc;		//

	memcpy(&data[5], frame->data, frame->can_dlc);					//SAVE MEMORY FOR DATA WITH MAX LENGTH DLC

	mcp_set_registers(tx_sidh[txbn], data, 5 + frame->can_dlc);		//SEND DATA TO REGISTERS OF TOTAL 14 BYTES WITH START REGISTER IS SIDH
	

	mcp_modify_register(tx_ctrl[txbn], 0x08, 0x08);					//MODIFY CANTX CTRL REGISTER --> TXREQ BIT TO 1

	uint8_t ctrl = mcp_read_register(tx_ctrl[txbn]);				//READ TXCTRL REGISTER TO CHECK ANY FLAG IS PRESENT.
	if ((ctrl & (0x40 | 0x20 | 0x10)) != 0)
	{
		return 4;
	}
	return 0;
}

/********************************************** Send message funtion used in main code *******************************************/


uint8_t sendMessage(const struct can_frame *frame)
{
	if (frame->can_dlc > 8) {		//IF FRAME LENGTH IS MORE THAN 8 THEN RETURN AND DO NOT SEND DATA
		return 4;
	}

	uint8_t txBuffers[3] = {0, 1, 2};		//MCP2515 SEND 3 BUFFERS
	uint8_t tx_ctrl[3] = {0x30,0x40,0x50};	//TXCNTRL REGISTERS

	for (int i=0; i<3; i++)
	{
		//const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
		uint8_t ctrlval = mcp_read_register(tx_ctrl[i]);	// READ ALL TXCTRL --> TXREQ BIT TO CHECK STATUS OF MESSAGE BUFF.
		if ( (ctrlval & 0x08) == 0 )		// IF TXREQ IS 0 THEN WE CAN SEND DATA.
		{
			
			return sendMessagek(txBuffers[i], frame);			//send data to each transmit buffer one by one
		}
	}

	return 2;					//else return nomsg sent
}

/****************************************** Send data with id and frame  *************************************************************/

void send_data(unsigned long id, unsigned char l_dlc,char *str)
{
	struct can_frame canMsg1;							// define data frame to be send
	canMsg1.can_id  = id;								// id of can bus data
	canMsg1.can_dlc = l_dlc;						// data length

	for(size_t i = 0; i < canMsg1.can_dlc; i++)
	{

		canMsg1.data[i] = str[i];				// copy string length to can data frame
		
	}
	
	sendMessage(&canMsg1);								// send msg
	
	
}

/****************************************** Receive data with id and frame  *************************************************************/


// void rcv_data()
// {
// 	struct can_frame canMsg;			// define data frame to be receive
// 	char buff1[20];
// 	char buff2[20];
// 	
// 	readmsg(&canMsg);					//read can Msg in canMsg struct variable
// 	
// // 	USART1_sendInt(canMsg.can_id);
// // 	
// // 	sprintf(buff1,"%d",canMsg.can_dlc);
// // 	USART1_sendString(buff1);
// 	
// 	_delay_ms(20);
// 	
// 	for(uint8_t i =0;i<2;i++)
// 	{
// 		sprintf(buff2,"%c",canMsg.data[i]);
// 		//USART1_sendString(buff2);
// 	}
// 	
// 	_delay_ms(100);
// 	
// 	//USART1_sendChar(' ');
// 	USART1_sendChar('\n');
// 	
// 	can_rcv_vle = ((canMsg.data[0]-48)*10)+(canMsg.data[1]-48);
// //	USART1_sendInt(can_rcv_vle);
// 	
//  	if(can_rcv_vle==45)
//  	{
//  		USART1_sendString("NOT DETECTED");
// 
//  	}
//  	
//  	else
// 	{
//  		USART1_sendString("S1 DETECTED");
//  		
//  		
// 	}
// 	
// }


/****************************************** Filter data[4] frame to get mode of et_cuff  *************************************************************/



uint8_t data_filter_mode(const unsigned char data)
{
	uint8_t data_mode=1;
	
	
	
	if(data>=48 && data<=57)
	{
		data_mode = data-48;
		
		
	}
	
	return data_mode;
	
}

/****************************************** clear send string with null  *************************************************************/



void strset(char *str)
{
	for(uint8_t i =0;i<strlen(str);i++)
	{
		str[i]="\0";
	}
}
#endif
