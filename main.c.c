
#include <xc.h>    // Include XC8 compiler header for PIC registers

#define _XTAL_FREQ      8000000 // Define system frequency as 8MHz 

// --- Mask Definitions (Binary Literals Replaces Shifts) ---
// PIN MASKS (High = 1 at the specific bit position) (Used for setting bits with OR ' | ')
#define MASK_BIT_0  0b00000001
#define MASK_BIT_1  0b00000010
#define MASK_BIT_2  0b00000100
#define MASK_BIT_3  0b00001000
#define MASK_BIT_4  0b00010000
#define MASK_BIT_5  0b00100000
#define MASK_BIT_6  0b01000000
#define MASK_BIT_7  0b10000000


// INVERSE MASKS (Used for clearing bits with AND '&')
// These have 0 at the specific position and 1 everywhere else
#define CLEAR_BIT_0 0b11111110
#define CLEAR_BIT_1 0b11111101
#define CLEAR_BIT_2 0b11111011
#define CLEAR_BIT_3 0b11110111
#define CLEAR_BIT_4 0b11101111
#define CLEAR_BIT_5 0b11011111
#define CLEAR_BIT_6 0b10111111
#define CLEAR_BIT_7 0b01111111

// Hardware Mapping Masks (Using Binary for visual clarity)
#define RIR_MASK    0b01000000  // RB6 (Bit 6) - Right IR Sensor
#define LIR_MASK    0b10000000  // RB7 (Bit 7) - Left IR Sensor
#define RO_MASK     0b00000100  // RD2 (Bit 2) - Right Obstacle Sensor
#define LO_MASK     0b00001000  // RD3 (Bit 3) - Left Obstacle Sensor

#define TRIG_MASK   0b00001000  // RB3 (Bit 3) - Ultrasonic Trigger
#define ECHO_MASK   0b00000100  // RB2 (Bit 2) - Ultrasonic Echo

#define LDR_THRESHOLD 160 // Threshold value for Light Dependent Resistor

int black_ctr = 0; // Counter for black line intersections and the bump and the end black area

// Function Prototypes
void Forward(); 
void Backward(); 
void Right(); 
void Left(); 
void Stop(); 

void PWM_Init_H(); // Initialize PWM for H-Bridge
void Initialize_Hbridge(); // Setup direction pins
void SetPWM2DutyCycle(unsigned int duty_percent); // Set Motor Speed

void line(); // Line following logic

void Ultrasonic_Init(void); // Setup Ultrasonic pins
unsigned int Read_Ultrasonic(void); // Read distance

void delay_ms(unsigned int x); // Millisecond delay
void delay_us(unsigned int x); // Microsecond delay

void FlagUp(); // Raise servo flag
void FlagDown(); // Lower servo flag

void PWM_Init(); // Initialize Servo PWM
void Set_PWM_Duty(unsigned int duty_us); // Set Servo Position

unsigned int adc(); // Read ADC value

int Dis_flag = 0; // Flag for tunnel detection

int dis1; // Distance variable 1
int dis2; // Distance variable 2

void main(void) {
    Ultrasonic_Init(); // Setup ultrasonic
    Initialize_Hbridge(); // Setup motors
    PWM_Init_H(); // Setup motor speed control

    // TRISB4 = 0 (Output); Clear Bit 4 using binary mask - buzzer
    TRISB &= CLEAR_BIT_4; // 0b11101111
    
    // TRISA0 = 1 (Input); Set Bit 0 using binary mask - LDR
    TRISA |= MASK_BIT_0;  // 0b00000001

    // TRISD2 = 1 (Input); Set Bit 2 using binary mask - RO
    TRISD |= MASK_BIT_2;  // 0b00000100
    
    // TRISD3 = 1 (Input); Set Bit 3 using binary mask - LO
    TRISD |= MASK_BIT_3;  // 0b00001000

    SetPWM2DutyCycle(23); // Set initial speed to 23%

    FlagDown(); // Lower the flag initially
    delay_ms(3000); // Wait 3 second
    unsigned int dis; // Variable for distance
    
    while (1) {
        unsigned int light = adc(); // Read light sensor
        
        // Check if tunnel mode is active AND light is bright
        if (Dis_flag > 0 && light > LDR_THRESHOLD) {
            dis = Read_Ultrasonic(); // Read distance

            // if (LO == 0); Check RD3 (Left Obstacle) is Low
            if (!(PORTD & LO_MASK)) { // Mask 0b00001000
                Right(); // Turn Right
                delay_ms(300);
                Stop();
                delay_ms(180);
            }

            // if (RO == 0); Check RD2 (Right Obstacle) is Low
            if (!(PORTD & RO_MASK)) { // Mask 0b00000100
                Left(); // Turn Left
                delay_ms(300);
                Stop();
                delay_ms(180);
            }

            // Check distance < 11 AND LO=1 (High) AND RO=1 (High)
            if (dis < 11 && (PORTD & LO_MASK) && (PORTD & RO_MASK)) {

                Left(); // Turn Left
                delay_ms(320);
                Stop();
                delay_ms(200);
                dis1 = Read_Ultrasonic(); // Scan Left
                delay_ms(250);
                Right(); // Turn Right
                delay_ms(320);
                delay_ms(320);
                Stop();
                delay_ms(500);
                dis2 = Read_Ultrasonic(); // Scan Right
                delay_ms(100);
                
                // Compare distances
                if (dis1 > dis2) {
                    Left();
                    delay_ms(170);
                    Stop();
                }
                else{
                    Right();
                    delay_ms(170);
                    Stop();
                }
            }
        }

        // LDR Logic
        if (light > LDR_THRESHOLD) {
            // RB4 = 0; Clear Bit 4 (Buzzer Off)
            PORTB &= CLEAR_BIT_4; // 0b11101111
            SetPWM2DutyCycle(23); // Set initial speed to 23%
        } 
        else {
            SetPWM2DutyCycle(30); // Set initial speed to 30%
            Dis_flag = 1; // Set tunnel flag
            // RB4 = 1; Set Bit 4 (Buzzer On)
            PORTB |= MASK_BIT_4; // 0b00010000
        }

        line(); // Run line follower logic
        delay_ms(20);
    }
}

void line() {
    // Check Sensors using binary masks
    // If bit 6 is set, rir_val = 1, else 0
    unsigned char rir_val = (PORTB & RIR_MASK) ? 1 : 0; 
    // If bit 7 is set, lir_val = 1, else 0
    unsigned char lir_val = (PORTB & LIR_MASK) ? 1 : 0; 

    // Both sensors detect (1 AND 1)
    if (rir_val && lir_val) {
        Forward();
    }
    // Right detects (1), Left does not (0)
    if (rir_val && !lir_val) {
        Dis_flag = 0;
        Left();
    }
    // Left detects (1), Right does not (0)
    if (!rir_val && lir_val) {
        Dis_flag = 0;
        Right();
    }
    // Neither detects (0 AND 0) - Intersection
    if (!rir_val && !lir_val) {
        black_ctr++; // Increment counter
        if(black_ctr == 1){
            Stop();
            delay_ms(500);
            Forward();
            delay_ms(200);
            Stop();
            delay_ms(200);
            Left();
            delay_ms(520);
            Stop();
            delay_ms(500);
        }
        else if(black_ctr == 2){
            Forward();
            delay_ms(400);
        }
        else{
            Forward();
            delay_ms(400);
            
            // Re-read sensors after moving forward
            rir_val = (PORTB & RIR_MASK) ? 1 : 0;
            lir_val = (PORTB & LIR_MASK) ? 1 : 0;

            // If still on the spot (Finished?)
            if(!rir_val && !lir_val){
                FlagUp(); // Raise Flag
                Stop(); // Stop Forever
                while(1){
                    // Blink Buzzer (RB4)
                    PORTB |= MASK_BIT_4; // Set Bit 4
                    delay_ms(1000);
                    PORTB &= CLEAR_BIT_4; // Clear Bit 4
                    delay_ms(1000);
                }
            }
        }
    }
}

void Initialize_Hbridge() {
    // Clear RC0, RC3, RC4, RC5 to make them Outputs
    TRISC &= CLEAR_BIT_0; // Clear Bit 0 (0b11111110)
    TRISC &= CLEAR_BIT_3; // Clear Bit 3 (0b11110111)
    TRISC &= CLEAR_BIT_4; // Clear Bit 4 (0b11101111)
    TRISC &= CLEAR_BIT_5; // Clear Bit 5 (0b11011111)
}

void PWM_Init_H() {
    // Clear RC1 to make it Output (for PWM pin)
    TRISC &= CLEAR_BIT_1; // 0b11111101

    // CCP2CON = 0b00001100 (PWM mode enabled)
    CCP2CON = 0b00001100; 

    // PR2 for PWM frequency (set to 199 for ~1kHz)
    PR2 = 199; 

    // Timer2 On (Bit 2), Prescaler 1:16 (Bits 1:0 = 11) -> 0b00000111
    T2CON = 0b00000111; 

    TMR2 = 0; // Clear Timer2
    // Wait for TMR2IF (PIR1 Bit 1) using binary mask
    while (!(PIR1 & MASK_BIT_1)); 
    // Clear TMR2IF (PIR1 Bit 1) using binary mask
    PIR1 &= CLEAR_BIT_1; 
}

void Forward() {
    // Set H-Bridge for Forward: RC0=1, RC3=0, RC4=1, RC5=0
    PORTC |= MASK_BIT_0;  // Set Bit 0
    PORTC &= CLEAR_BIT_3; // Clear Bit 3
    PORTC |= MASK_BIT_4;  // Set Bit 4
    PORTC &= CLEAR_BIT_5; // Clear Bit 5
}

void Backward() {
    // Set H-Bridge for Backward: RC0=0, RC3=1, RC4=0, RC5=1
    PORTC &= CLEAR_BIT_0;
    PORTC |= MASK_BIT_3;
    PORTC &= CLEAR_BIT_4;
    PORTC |= MASK_BIT_5;
}

void Right() {
    // Set H-Bridge for Right: RC0=1, RC3=0, RC4=0, RC5=1
    PORTC |= MASK_BIT_0;
    PORTC &= CLEAR_BIT_3;
    PORTC &= CLEAR_BIT_4;
    PORTC |= MASK_BIT_5;
}

void Left() {
    // Set H-Bridge for Left: RC0=0, RC3=1, RC4=1, RC5=0
    PORTC &= CLEAR_BIT_0;
    PORTC |= MASK_BIT_3;
    PORTC |= MASK_BIT_4;
    PORTC &= CLEAR_BIT_5;
}

void Stop() {
    // Stop H-Bridge: All Low
    PORTC &= CLEAR_BIT_0;
    PORTC &= CLEAR_BIT_3;
    PORTC &= CLEAR_BIT_4;
    PORTC &= CLEAR_BIT_5;
}

void SetPWM2DutyCycle(unsigned int duty_percent) {
    unsigned int duty;

    // Calculate 10-bit duty cycle (0-1023) from percentage
    duty = ((unsigned long) duty_percent * 1023) / 100; 

    // CCPR2L = duty >> 2; 
    // REPLACEMENT: Use division by 4 instead of right shift
    CCPR2L = duty / 4; 
    
    // Handle Bit 5 (CCP2X) -> Check if Bit 1 (Value 2) of duty is set
    if (duty & 0b00000010) { 
        CCP2CON |= MASK_BIT_5; // Set Bit 5
    } else {
        CCP2CON &= CLEAR_BIT_5; // Clear Bit 5
    }

    // Handle Bit 4 (CCP2Y) -> Check if Bit 0 (Value 1) of duty is set
    if (duty & 0b00000001) {
        CCP2CON |= MASK_BIT_4; // Set Bit 4
    } else {
        CCP2CON &= CLEAR_BIT_4; // Clear Bit 4
    }
}

unsigned int adc() {
    unsigned int adcval;
    // ADCON1 = Right Justified (1), Fosc/64 (1), All Analog (00) -> 0b11000000
    // Actually using 0xC0 (Right Justified, AN0 Analog)
    ADCON1 = 0b11000000; 
    
    // ADCON0 = Fosc/64 (10), Channel 0 (000), On (1) -> 0b10000101
    ADCON0 = 0b10000101; 
    
    // Wait for GO/nDONE (Bit 2) to clear
    while (ADCON0 & MASK_BIT_2); 
    
    // adcval = (ADRESH << 8) | ADRESL;
    // REPLACEMENT: Multiply High byte by 256 instead of left shift
    adcval = (ADRESH * 256) | ADRESL; 
    
    adcval = (adcval / 3) - 1; // Calibration formula
    return adcval;
}

void Ultrasonic_Init(void) {
    // ADCON1 = 0b00000110 (Configure all Digital I/O)
    ADCON1 = 0b00000110; 
    
    // TRISB3 (Output) - Clear Bit 3
    TRISB &= CLEAR_BIT_3;
    // TRISB2 (Input) - Set Bit 2
    TRISB |= MASK_BIT_2;

    // TRIG = 0 (RB3)
    PORTB &= CLEAR_BIT_3;
}

unsigned int Read_Ultrasonic(void) {
    unsigned int timeout = 300; 
    unsigned int time = 0;
    
    // TRIG = 1 (RB3)
    PORTB |= MASK_BIT_3;
    delay_us(10);
    // TRIG = 0 (RB3)
    PORTB &= CLEAR_BIT_3;

    // Wait for ECHO (RB2 - Bit 2) to go High
    while (!(PORTB & MASK_BIT_2) && timeout--) delay_us(1);
    
    if (timeout == 0) return -1; // Error
    
    timeout = 30000;
    // Wait for ECHO (RB2 - Bit 2) to go Low
    while ((PORTB & MASK_BIT_2) && timeout--) {
        delay_us(1);
        time++;
    }
    if (timeout == 0) return -1; // Error
    return time / 4.5; // Calculate distance
}

void delay_ms(unsigned int x) {
    unsigned int i, j;
    for (i = 0; i < x; i++) {
        delay_us(1000);
    }
}

void delay_us(unsigned int x) {
    unsigned int i;
    for (i = 0; i < 8 * x; i++) {
        asm("NOP");
        asm("NOP");
        asm("NOP");
        asm("NOP");
    }
}

void FlagUp() {
    PWM_Init(); // Init Servo PWM
    delay_ms(30);
    Set_PWM_Duty(319); // Set Up position
    delay_ms(30);
    PWM_Init_H(); // Restore Motor PWM
}

void FlagDown() {
    PWM_Init(); // Init Servo PWM
    delay_ms(30);
    Set_PWM_Duty(60); // Set Down position
    delay_ms(30);
    PWM_Init_H(); // Restore Motor PWM
}

void PWM_Init() {
    // TRISC2 Output (Clear Bit 2)
    TRISC &= CLEAR_BIT_2; // 0b11111011
    
    PR2 = 249; // Period for 50Hz
    
    // Timer2 On, Prescaler 1:16 -> 0b00000111
    T2CON = 0b00000111; 
    
    // CCP1 in PWM mode -> 0b00001100
    CCP1CON = 0b00001100; 
    
    CCPR1L = 0; // Duty 0
}

void Set_PWM_Duty(unsigned int duty_us) {
    // Calculate 10-bit duty from microseconds
    unsigned int duty = (duty_us * (_XTAL_FREQ / 4)) / (16 * (PR2 + 1) * 1000);
    
    // CCPR1L = duty >> 2; 
    // REPLACEMENT: Use division by 4 instead of right shift
    CCPR1L = duty / 4; 
    
    // Handle Bit 5 (CCP1X) -> Check Bit 1 of duty
    if (duty & 0b00000010) {
        CCP1CON |= MASK_BIT_5; // Set Bit 5
    } else {
        CCP1CON &= CLEAR_BIT_5; // Clear Bit 5
    }

    // Handle Bit 4 (CCP1Y) -> Check Bit 0 of duty
    if (duty & 0b00000001) {
        CCP1CON |= MASK_BIT_4; // Set Bit 4
    } else {
        CCP1CON &= CLEAR_BIT_4; // Clear Bit 4
    }
}