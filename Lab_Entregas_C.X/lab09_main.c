/*
 * Archivo:   lab09_main.c
 * Dispositivo: PIC16F887
 * Autor: Brandon Garrido 
 * 
 * Programa: Laboratorio 9 - modulo ccp
 * Hardware:Potenciómetros en PORTA y servos en PORTC
 * 
 * Creado: Abril 26, 2021
 * Última modificación: Abril 20, 2021
 */

//------------------------------------------------------------------------------
//                          Importación de librerías
//------------------------------------------------------------------------------    
#include <xc.h>

//------------------------------------------------------------------------------
//                          Directivas del compilador
//------------------------------------------------------------------------------
#define _XTAL_FREQ 8000000 //Para delay

//------------------------------------------------------------------------------
//                          Palabras de configuración
//------------------------------------------------------------------------------    
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT//Oscillator Selection bits(INTOSCIO 
                              //oscillator: I/O function on RA6/OSC2/CLKOUT pin, 
                              //I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled and 
                          //can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR  
                                //pin function is digital input, MCLR internally 
                                //tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                //protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                //protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                //Internal/External Switchover mode is disabled
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                //(Fail-Safe Clock Monitor is disabled)
#pragma config LVP = ON         //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out 
                                //Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
                                //(Write protection off)

//------------------------------------------------------------------------------
//                          Prototipos
//------------------------------------------------------------------------------
void setup(void);  //Configuración

//------------------------------------------------------------------------------
//                          Loop principal
//------------------------------------------------------------------------------
void main(void) {
    setup(); //Configuración
    ADCON0bits.GO = 1; //La conversión ADC se ejecuta
    while(1)
    {
        if(ADCON0bits.GO == 0){ //Si la conversión ya está terminada
            if (ADCON0bits.CHS == 0){ //Si está en el primer canal,
                ADCON0bits.CHS = 1;}  //pasa al segundo canal
            else {                   //Si está en el segundo canal,
                ADCON0bits.CHS = 0;} //se coloca en el primer canal
            
            __delay_us(50); //Delay para el capacitor sample/hold
            ADCON0bits.GO = 1; //Se vuelve a ejecutar la conversión ADC
        }
        }
    return;
}

void __interrupt() isr(void){
    
    if (PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0) { //Verifica el canal en l que se encuentra
            PORTB = ADRESH;
            CCPR1L = (PORTB>>1) + 128; //Swift y ajuste de señal
            CCP1CONbits.DC1B1 = PORTBbits.RB0;
            CCP1CONbits.DC1B0 = ADRESL>>7;}
        
        else{
            PORTB = ADRESH;
            CCPR2L = (PORTB>>1) + 128;//Swift y ajuste de señal
            CCP2CONbits.DC2B1 = PORTBbits.RB0;
            CCP2CONbits.DC2B0 = ADRESL>>7;}
        
        PIR1bits.ADIF = 0; //Se limpia la bandera de ADC
    }
    return;
}

//------------------------------------------------------------------------------
//                          Configuración
//------------------------------------------------------------------------------
void setup(){
    
    //Configuracion reloj
    OSCCONbits.IRCF2 = 1; //Frecuencia a 8MHZ
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    
    //Configurar entradas y salidas
    ANSELH = 0x00;//Pines digitales
    ANSEL = 0x03; //Primeros dos pines con entradas analógicas
    
    TRISA = 0x03; //Para entrada de los potenciometros
    TRISB = 0x00;
    TRISC = 0x00; //Para servos
               
    PORTA = 0x00; //Se limpian los puertos    
    PORTB = 0x00;
    PORTC = 0x00;
    
    //Configurar ADC
    ADCON1bits.ADFM = 0; //Justificar a la izquierda
    ADCON1bits.VCFG0 = 0; //Vss
    ADCON1bits.VCFG1 = 0; //VDD
    
    ADCON0bits.ADCS = 0b10; //ADC oscilador -> Fosc/32
    ADCON0bits.CHS = 0;     //Comenzar en primer canal
    __delay_us(50);        
    ADCON0bits.ADON = 1;    //Habilitar la conversión ADC
    
    //Configurar PWM
    PR2 = 250; //Valor inicial de PR2
    CCP1CONbits.P1M = 0; //PWM bits de salida
    CCP1CONbits.CCP1M = 0b00001100; //Se habilita PWM   
    CCP2CONbits.CCP2M = 0b00001100;   
    
    CCPR1L = 0x0F; 
    CCPR2L = 0x0F;
    CCP1CONbits.DC1B = 0; //Bits menos significativos del Duty Cycle
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
    
    PIR1bits.TMR2IF = 0; //Se limpia la bandera
    T2CONbits.T2CKPS1 = 1; //Prescaler de 16
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1; //Se enciende el TMR2
    
    while (!PIR1bits.TMR2IF); //Se espera una interrupción
    PIR1bits.TMR2IF = 0;
            
    //Configurar la interrupcion
    INTCONbits.GIE = 1;  //Enable interrupciones globales
    INTCONbits.PEIE = 1; //Enable interrupciones periféricas
    PIE1bits.ADIE = 1;   //Enable interrupción ADC
    PIR1bits.ADIF = 0;   //Se limpia bandera de interrupción ADC
    return;
 } 
