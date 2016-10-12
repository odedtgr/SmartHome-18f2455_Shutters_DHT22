#ifndef MAIN_C
#define MAIN_C

// includes
#include <string.h>
#include <usart.h>
#include <delays.h>
#include <stdio.h>
#include <math.h>
#include <EEP.h>
#include "config.h"

//define
//=================================
// Comment out the following line if you do not want the debug
// feature of the firmware (saves code and RAM space when off)//
// Note: if you use this feature you must compile with the large
// memory model on (for 24-bit pointers) so that the sprintf()
// function will work correctly.  If you do not require debug it's
// recommended that you compile with the small memory model and 
// remove any references to <strings.h> and sprintf().
#define DEBUGON

//Use LATA to write, PORTA to read.
//Remember to configure input/output
#define LED1        LATAbits.LATA3 //Status led
//shutter pins
#define upRelay     LATAbits.LATA4
#define downRelay	LATAbits.LATA5 
#define upSwitch	PORTBbits.RB6 //Switch 1 input
#define downSwitch	PORTBbits.RB7 //Switch 2 input
//DHT22 pins
#define DHT22in     PORTBbits.RB5 //DHT22  input
#define DHT22out    LATBbits.LATB5 //DHT22  output

#define ADDRESS 2//XBee address.

// Define the globals 
#pragma udata
int deviceState = 0;
char ADDRH = 0x00;
char ADDRL = 0x00;
//UART variables
char RxBuffer[100];
char TxBuffer[100];
int receivePos = 0;
//shutter variables
double time = 0;
double SWcount = 0;
unsigned char upBuffer = 0;
unsigned char downBuffer = 0;
//DHT22 variables
unsigned char Check, T_byte1, T_byte2, RH_byte1, RH_byte2, Ch, Sum;
unsigned Temp, TempPrev, RH, RHPrev;
double secondsCounter=0;
#pragma udata

// Private function prototypes
static void initialisePic(void);
void highPriorityISRCode();
void lowPriorityISRCode();
void initUsart(void);
void initXbee(void);
void ReceiveUsart(void);
void TransmitUsart(char device, char data);
void TransmitUsartAT(void);
void delay1sec(void);
void handleMessge(void);
void shutterCommands(char device);
void moveToPos(int pos, char device);
void LimitDeviceState(void);
//DHT22 prototypes
void StartSignal();
void CheckResponse();
char ReadData();
void ReadDHT22();
void TransmitUsartTemp(char device);

#pragma code HIGH_INTERRUPT_VECTOR = 0x08

void High_ISR(void) {
    _asm goto highPriorityISRCode _endasm
}

#pragma code LOW_INTERRUPT_VECTOR = 0x18

void Low_ISR(void) {
    _asm goto lowPriorityISRCode _endasm
}

#pragma code

// High-priority ISR handling function
#pragma interrupt highPriorityISRCode

void highPriorityISRCode() {
    // Application specific high-priority ISR code goes here

    //USART receive interrupt, Cleared when RCREG is read.
    if (PIR1bits.RCIF) {
        ReceiveUsart();
    }
    //PORTB interrupt on change
    if (INTCONbits.RBIF) {
        //Switch pressed
        if ((upSwitch == 0) | (downSwitch == 0)) {
            INTCONbits.RBIE = 0; //Disable interrupt on change
            T1CONbits.TMR1ON = 1; //Timer1 On bit
            INTCONbits.GIEL = 1; //Enable low priority interrupts
        }

        INTCONbits.RBIF = 0; //Flag must be cleared
    }
}

// Low-priority ISR handling function
#pragma interruptlow lowPriorityISRCode

void lowPriorityISRCode() {
    int threshhold;

    //TMR1 interrupt on overflow
    if (PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0; //clear the interrupt flag
        TMR1H = 0xC1;
        TMR1L = 0x80; //Pre-scaler 1:1; TMR1 Pre-load = 49536; Actual Interrupt Time : 8 ms

        upBuffer = (upBuffer << 1) | !upSwitch;
        downBuffer = (downBuffer << 1) | !downSwitch;

        //rising edge detected
        if ((upBuffer == 0b01111111) | (downBuffer == 0b01111111)) {
            INTCONbits.GIEL = 1; //Enable low priority interrupts
            INTCONbits.RBIF = 0; //Flag must be cleared            
            INTCONbits.RBIE = 1; //Enable interrupt on change
            SWcount = 0; //reset the time counter
            if ((upRelay == 0)&(downRelay == 0)) {
                if (!upSwitch)
                    moveToPos(100, 1);
                if (!downSwitch)
                    moveToPos(0, 1);
            } else {
                if (upRelay)
                    deviceState = deviceState + (int) (time / (double) Read_b_eep(0x01)*100);
                else
                    deviceState = deviceState - (int) (time / (double) Read_b_eep(0x02)*100);
                upRelay = 0;
                downRelay = 0;
                LimitDeviceState();
                TransmitUsart(1, (char) deviceState);
            }
        }
        //de-bounced ON
        if ((upBuffer == 0xff) | (downBuffer == 0xff))
            SWcount++; // count timer ticks of de-bounced ON. 8ms

        //falling edge
        if ((upBuffer == 0b11111110) | (downBuffer == 0b11111110)) {
            T1CONbits.TMR1ON = 0; //Timer1 On bit
            INTCONbits.GIEL = 1; //Enable low priority interrupt
            INTCONbits.RBIF = 0; //Flag must be cleared
            INTCONbits.RBIE = 1; //Enable interrupt on change

            // Handle output according to time button was pressed
            threshhold = 1 * 1000 / 8; //1 sec in timer1 counts
            if (SWcount > threshhold) {
                //long button press - Manual mode
                if (upRelay == 1)
                    deviceState = deviceState + (int) (SWcount * 8.192 / 1000 / (double) Read_b_eep(0x01) * 100);
                else
                    deviceState = deviceState - (int) (SWcount * 8.192 / 1000 / (double) Read_b_eep(0x02) * 100);
                upRelay = 0;
                downRelay = 0;
                LimitDeviceState();
                TransmitUsart(1, (char) deviceState);
            }
        }
    }
    
    //TMR0 interrupt on overflow - Read DHT22 interval
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0; //clear the interrupt flag
        TMR0H = 0x0B;
        TMR0L = 0xDC; //TMR0 Preload = 3036; Actual Interrupt Time : 1 sec
        if (secondsCounter < 1){
            secondsCounter++;
        }else {
            //time has passed
            secondsCounter=0;
            ReadDHT22();
        }  
    }
}

// Main program entry point

void main(void) {
    //init device
    initialisePic();
    initUsart();
    LATAbits.LATA1 = 1; //power DHT22

    //init EEPROM with default values
    if (Read_b_eep(0x01) == 0xff)
        Write_b_eep(0x01, (char) 14);
    if (Read_b_eep(0x02) == 0xff)
        Write_b_eep(0x02, (char) 14);
    if (Read_b_eep(0x03) == 0xff)
        Write_b_eep(0x03, (char) 2);

    initXbee();
    //Main loop
    while (1) {
        Sleep();
    }
}

void handleMessage(void) {
    if (RxBuffer[3] == 0x81) //RX (Receive) Packet: 16-bit Address
    {
        ADDRH = RxBuffer[4];
        ADDRL = RxBuffer[5];
        switch ((int) RxBuffer[8]) //first data byte, Device selection
        {
            case 1://Device is shutter (livingroom)
                shutterCommands(RxBuffer[8]);
                break;

            default: // Unknown command received
                break;
        }
    }
}

void shutterCommands(char device) {
    INTCONbits.GIEH = 1; //enable high priority interrupts. enable stopping shutters while on delay.
    switch ((int) RxBuffer[9]) //Device specific command
    {
        case 0://get device status
            TransmitUsart(device, (char) deviceState);
            break;

        case 1://move to position
            moveToPos((int) RxBuffer[10], device);
            break;

        case 2://stop
            if (upRelay == 1)
                deviceState = deviceState + (int) (time / (double) Read_b_eep(0x01) * 100);
            else
                deviceState = deviceState - (int) (time / (double) Read_b_eep(0x02) * 100);
            upRelay = 0;
            downRelay = 0;
            LimitDeviceState();
            TransmitUsart(device, (char) deviceState);
            break;

        case 3://Write EEPROM parameter
            Write_b_eep(RxBuffer[10], RxBuffer[11]);
            break;

        case 4://Read EEPROM parameter
            TransmitUsart(RxBuffer[10], Read_b_eep(RxBuffer[10]));
            break;

        default: // Unknown command received
            break;
    }
}

void moveToPos(int pos, char device) {
    double moveTime;

    if (pos == 100) {
        moveTime = (double) Read_b_eep(0x01) + (double) Read_b_eep(0x03);
        downRelay = 0;
        Delay10KTCYx(20); //0.1 sec delay
        upRelay = 1;
    } else if (pos == 0) {
        moveTime = (double) Read_b_eep(0x02) + (double) Read_b_eep(0x03);
        upRelay = 0;
        Delay10KTCYx(20); //0.1 sec delay
        downRelay = 1;
    } else {
        moveTime = fabs((double) (pos - deviceState) / 100);
        if (pos > deviceState) {//moving up
            moveTime = moveTime * (double) Read_b_eep(0x01);
            downRelay = 0;
            Delay10KTCYx(20); //0.1 sec delay
            upRelay = 1;
        } else {
            moveTime = moveTime * (double) Read_b_eep(0x02);
            upRelay = 0;
            Delay10KTCYx(20); //0.1 sec delay
            downRelay = 1;
        }
    }

    for (time = 0; time < moveTime; time = time + 0.1) {
        Delay10KTCYx(20); //0.1 sec delay
        if ((upRelay == 0)&(downRelay == 0))
            return;
    }

    upRelay = 0;
    downRelay = 0;
    deviceState = pos;
    TransmitUsart(device, (char) deviceState);
}

// Initialise the PIC
static void initialisePic(void) {
    //Internal clock freq 8MHz
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    //sleep configuration
    OSCCONbits.IDLEN = 1;
    // Default all pins to digital
    ADCON1 = 0x0F;
    // Clear all ports
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000;
    // Configure ports as inputs (1) or outputs(0)
    TRISA = 0b00000000;
    TRISB = 0b11100000;
    TRISC = 0b00000000;
    // Configure interrupts
    INTCON2bits.RBPU = 0; //PORTB weak pullup enabeled
    INTCONbits.GIEH = 1; //Enable global interrupts
    INTCONbits.GIEL = 1; //Enable low priority interrupts
    RCONbits.IPEN = 1; //Enable priority interrupts
    PORTB = PORTB;
    INTCONbits.RBIF = 0; //Reset the interrupt flag
    INTCON2bits.RBIP = 1; //PORTB interrupt priority set to High
    INTCONbits.RBIE = 1; //enable interrupt on input change on PORTB<7:4>
    // Timer1 configuration
    T1CONbits.RD16 = 1;
    PIR1bits.TMR1IF = 0; //clear the interrupt flag
    TMR1H = 0xC1;
    TMR1L = 0x80; //Pre-scaler 1:1; TMR1 Pre-load = 49536; Actual Interrupt Time : 8 ms
    IPR1bits.TMR1IP = 0; //0 = Low priority interrupt
    PIE1bits.TMR1IE = 1; ///TMR1 interrupt enabled
    // Timer0 configuration - DHT22 read interval
    T0CONbits.T0CS = 0; //0 = Internal instruction cycle clock (CLKO)
    T0CONbits.T08BIT = 0;//0=16 bit mode
    T0CONbits.PSA = 0; //Use pre-scalar
    T0CONbits.T0PS0 = 0;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS2 = 1; //prescalar 1:32, overflow every 8.192ms
    TMR0H	 = 0x0B;
    TMR0L	 = 0xDC;//TMR0 Preload = 3036; Actual Interrupt Time : 1 sec
    INTCON2bits.T0IP = 0; //TMR0 interrpt low priority
    INTCONbits.TMR0IE = 1; //Timer0 interrupt on overflow
}

void initUsart() {
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;
    TXSTAbits.TXEN = 1; // Transmit Enable bit
    RCSTAbits.SPEN = 1; // Serial Port Enable bit
    RCSTAbits.CREN = 1; // Continuous Receive Enable bit
    TXSTAbits.BRGH = 1; // High Baud Rate Select bit
    BAUDCONbits.BRG16 = 1; //16-Bit Baud Rate Register Enable bit
    SPBRGH = 0;
    SPBRG = 103; // baudrate: 103-19,230 207-9,600
    IPR1bits.RCIP = 1; //Set USART receive interrupt low priority
    PIE1bits.RCIE = 1; //Enable USART receive interrupts
    PIR1bits.TXIF = 1; //The EUSART transmit buffer, TXREG, is empty (cleared when TXREG is written)
}

void initXbee() {
    delay1sec();
    sprintf(TxBuffer, "X");
    TransmitUsartAT();
    delay1sec();
    sprintf(TxBuffer, "+++"); //Enter At command mode
    TransmitUsartAT();
    delay1sec();
    sprintf(TxBuffer, "ATMY%d\r", ADDRESS); //Set address
    TransmitUsartAT();
    sprintf(TxBuffer, "ATBD4\r"); //Set BaudeRate 19,200
    TransmitUsartAT();
    sprintf(TxBuffer, "ATAP1,WR,AC,CN\r"); //Set API mode 1
    TransmitUsartAT();
}

void delay1sec() {
    //TCY = 1sec/(8MHz/4) =0.5 us
    Delay10KTCYx(220); //1 sec is 200, but AT commands did not work, so i increased the delay.
}

void ReceiveUsart() {
    char c;
    int i;
    if (PIR1bits.RCIF == 1 && PIE1bits.RCIE == 1) //Check data in RCREG.
    {
        if (RCSTAbits.OERR == 1) {
            RCSTAbits.CREN = 0x0; //Stop continuous reception to clear the error flag FERR.
            RCSTAbits.CREN = 0x1; //Enable continuous reception again.
        }
        c = RCREG; //Read data from RCREG
        if (c == 0x7e)//API Frame start delimeter received
            receivePos = 0;
        RxBuffer[receivePos] = c;
        //length of data received
        if (receivePos >= 2) {
            if (receivePos == RxBuffer[1]*256 + RxBuffer[2] + 3) {
                handleMessage();
            }
        }
        receivePos++;
    }
}

void TransmitUsart(char device, char data) {
    int i, len;

    TxBuffer[0] = 0x7e; //Start delimiter
    TxBuffer[1] = 0x00; //length MSB
    TxBuffer[2] = 0x07; //length LSB
    TxBuffer[3] = 0x01; //API identifier: TX Request, 16-bit address
    TxBuffer[4] = 0x00; //frame ID
    TxBuffer[5] = ADDRH; //Destination address MSB
    TxBuffer[6] = ADDRL; //Destination address LSB
    TxBuffer[7] = 0x00; //options -Disable ACK
    TxBuffer[8] = device; //Device selection
    TxBuffer[9] = data; //Data
    TxBuffer[10] = 0xff - (TxBuffer[3] + TxBuffer[4] + TxBuffer[5] + TxBuffer[6] + TxBuffer[7] + TxBuffer[8] + TxBuffer[9]); //checksum

    len = TxBuffer[1]*256 + TxBuffer[2] + 4;
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}

void TransmitUsartAT() {
    int i, len;
    len = strlen(TxBuffer);
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}

void LimitDeviceState() {
    if (deviceState < 0)
        deviceState = 0;
    if (deviceState > 100)
        deviceState = 100;
}

//////////////DHT22 functions////////////////
void StartSignal() {
    TRISBbits.RB5 = 0; //Configure RB5 as output
    DHT22out = 0;
    Delay1KTCYx(4); //2ms
    DHT22out = 1; //sends 1 to the sensor
    Delay10TCYx(5); //25us
    TRISBbits.RB5 = 1; //Configure as input
}

void CheckResponse() {
    Check = 0;
    Delay10TCYx(6); //40us
    if (DHT22in == 0) {
        Delay10TCYx(16); //80us
        if (DHT22in == 1)
            Check = 1;
        Delay10TCYx(8); //40us
    }
}

char ReadData() {
    char i, j;
    for (j = 0; j < 8; j++) {
        while (!DHT22in); //Wait until PORTD.F0 goes HIGH
        Delay10TCYx(6); //30us
        if (DHT22in == 0)
            i &= ~(1 << (7 - j)); //Clear bit (7-b)
        else {
            i |= (1 << (7 - j)); //Set bit (7-b)
            while (DHT22in);
        } //Wait until PORTD.F0 goes LOW
    }
    return i;
}

void ReadDHT22() {
    INTCONbits.RBIE = 0; //interrupt on input change on PORTB<7:4>
    StartSignal();
    CheckResponse();
    if (Check == 1) {
        RH_byte1 = ReadData();
        RH_byte2 = ReadData();
        T_byte1 = ReadData();
        T_byte2 = ReadData();
        Sum = ReadData();
        if (Sum == (char) (RH_byte1 + RH_byte2 + T_byte1 + T_byte2)) {
            Temp = T_byte1;
            Temp = (Temp << 8) | T_byte2;
            RH = RH_byte1;
            RH = (RH << 8) | RH_byte2;
            if (Temp > 0X8000) {
                Temp = Temp & 0X7FFF;
            }
            //sprintf(TxBuffer, "TEMP=%d, RH=%d", Temp, RH); //Set address
            //TransmitUsartAT();
            if ((fabs((int)Temp-(int)TempPrev)>=2)|(fabs((int)RH-(int)RHPrev)>=50)) {
                TempPrev=Temp;
                RHPrev=RH;
                TransmitUsartTemp(2);
            }
        }
    }
    INTCONbits.RBIE = 1; //interrupt on input change on PORTB<7:4>
}

void TransmitUsartTemp(char device) {
    int i, len;

    TxBuffer[0] = 0x7e; //Start delimiter
    TxBuffer[1] = 0x00; //length MSB
    TxBuffer[2] = 0x0a; //length LSB
    TxBuffer[3] = 0x01; //API identifier: TX Request, 16-bit address
    TxBuffer[4] = 0x00; //frame ID
    TxBuffer[5] = ADDRH; //Destination address MSB
    TxBuffer[6] = ADDRL; //Destination address LSB
    TxBuffer[7] = 0x00; //options -Disable ACK
    TxBuffer[8] = device; //Device selection
    TxBuffer[9] = T_byte1; //Data
    TxBuffer[10] = T_byte2; //Data
    TxBuffer[11] = RH_byte1; //Data
    TxBuffer[12] = RH_byte2; //Data
    TxBuffer[13] = 0xff - (char) (TxBuffer[3] + TxBuffer[4] + TxBuffer[5] + TxBuffer[6] + TxBuffer[7] + TxBuffer[8] + TxBuffer[9] + TxBuffer[10] + TxBuffer[11] + TxBuffer[12]);
    len = TxBuffer[1]*256 + TxBuffer[2] + 4;
    for (i = 0; i < len; i++) {
        while (TXSTAbits.TRMT != 1);
        WriteUSART(TxBuffer[i]);
    }
}