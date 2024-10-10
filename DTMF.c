#include <stdio.h>
#include <math.h>
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#define SAMP_RATE 40000       // Sampling rate
#define SAMPLES 4000          // Number of samples to process the more samples the more accurate but the longer processing time
#define ADC_PIN 26            // GPIO pin for the ADC (ADC0 on GPIO26)
#define THRESHOLD 60000       // georetzel threshold
#define CONTROL_FREQUENCY 150 // Frequency to detect noise/human speech 
#define ROW1_FREQUENCY 697    // Frequency for 123A row
#define ROW2_FREQUENCY 770    // Frequency for 456B row 
#define ROW3_FREQUENCY 852    // Frequency for 789C row
#define ROW4_FREQUENCY 941    // Frequency for *0#D row 
#define COL1_FREQUENCY 1209   // Frequency for 147* col
#define COL2_FREQUENCY 1336   // Frequency for 2580 col
#define COL3_FREQUENCY 1477   // Frequency for 369# col
#define COL4_FREQUENCY 1633   // Frequency for ABCD col

//A Mapt of all DTMF Values
const char A_DTMF_MAP[4][4] = {{'1','2','3','A',},{'4','5','6','B'},{'7','8','9','C'},{'*','0','#','D'}};

//Collects samples and stores them into the given array
void collectSamples(__uint16_t *samples){
            for (int i = 0; i < SAMPLES; i++) {
            samples[i] = adc_read();
            /*
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
        sleep_ms(500);*/
            sleep_us(1000000 / SAMP_RATE);  // Delay for the sample rate
        }
}
    // Applies Goertzel Algorthim to the inputted samples returning the magnitude. It shows relatively how much the frequency is present in the signal.
__uint32_t goertzel(int numSamples, int targetFreq, __uint32_t sampleRate, __uint16_t* samples) {
    int k = (int)(0.5 + ((numSamples * targetFreq) / (float)sampleRate));
    float omega = (2.0 * 3.14159 * k) / numSamples;
    float sine = sin(omega);
    float cosine = cos(omega);
    float coeff = 2.0 * cosine;
    
    float Q0 = 0, Q1 = 0, Q2 = 0;
    
    for (int i = 0; i < numSamples; i++) {
        Q0 = coeff * Q1 - Q2 + samples[i];
        Q2 = Q1;
        Q1 = Q0;
    }

    float real = (Q1 - Q2 * cosine);
    float imag = (-Q2 * sine);
    
    // Magnitude of the frequency component
    float magnitude = sqrt(real * real + imag * imag);
    return (__uint32_t) magnitude;
}
// Uses Goeretzel to return the dtmf button pressed. Returns 0x00 if there is no detected signal.
char dtmfDecode(__uint16_t *samples){
    char c_result = 0x00;
    __uint32_t u32_controlMag= goertzel(SAMPLES, CONTROL_FREQUENCY, SAMP_RATE, samples);
    __uint32_t u32_mag = u32_controlMag * 1.5 < THRESHOLD ? THRESHOLD : u32_controlMag + THRESHOLD * 0.5;
    __uint32_t au32_rowMags[4] = {  goertzel(SAMPLES, ROW1_FREQUENCY, SAMP_RATE, samples),
                                    goertzel(SAMPLES, ROW2_FREQUENCY, SAMP_RATE, samples),
                                    goertzel(SAMPLES, ROW3_FREQUENCY, SAMP_RATE, samples),   
                                    goertzel(SAMPLES, ROW4_FREQUENCY, SAMP_RATE, samples)};
    // I trust the compiler to make this efficent
    if(au32_rowMags[0] > au32_rowMags[1] && au32_rowMags[0] > au32_rowMags[2] && au32_rowMags[0] > au32_rowMags[3])
        c_result = 0;
    else if(au32_rowMags[1] > au32_rowMags[2] && au32_rowMags[1] > au32_rowMags[3])
        c_result = 1;
    else if(au32_rowMags[2] > au32_rowMags[3])
        c_result = 2;
    else
        c_result = 3;
    
    // Make sure the max magnitude is at least the threshold and 40% higher than the next highest magnitude.
    if(au32_rowMags[c_result] > u32_mag && 
        (   (au32_rowMags[c_result] > 0.6 * au32_rowMags[0] || c_result == 0) && 
            (au32_rowMags[c_result] > 0.6 * au32_rowMags[1] || c_result == 1) && 
            (au32_rowMags[c_result] > 0.6 * au32_rowMags[2] || c_result == 2) && 
            (au32_rowMags[c_result] > 0.6 * au32_rowMags[3] || c_result == 3))){
            __uint32_t au32_colMags[4] = { goertzel(SAMPLES, COL1_FREQUENCY, SAMP_RATE, samples),
                                            goertzel(SAMPLES, COL2_FREQUENCY, SAMP_RATE, samples),
                                            goertzel(SAMPLES, COL3_FREQUENCY, SAMP_RATE, samples),   
                                            goertzel(SAMPLES, COL4_FREQUENCY, SAMP_RATE, samples)};
            __uint8_t u8_index;
            if(au32_colMags[0] > au32_colMags[1] && au32_colMags[0] > au32_colMags[2] && au32_colMags[0] > au32_colMags[3])
                u8_index = 0;
            else if(au32_colMags[1] > au32_colMags[2] && au32_colMags[1] > au32_colMags[3])
                u8_index = 1;
            else if(au32_colMags[2] > au32_colMags[3])
                u8_index = 2;
            else
                u8_index = 3;
            
            // Make sure the column magnitude is larger than threshold and larger than other magnitudes by 40% or more
            if(au32_colMags[u8_index] > u32_mag && 
                (   (au32_colMags[u8_index] > 0.6 * au32_colMags[0] || u8_index == 0) && 
                    (au32_colMags[u8_index] > 0.6 * au32_colMags[1] || u8_index == 1) && 
                    (au32_colMags[u8_index] > 0.6 * au32_colMags[2] || u8_index == 2) && 
                    (au32_colMags[u8_index] > 0.6 * au32_colMags[3] || u8_index == 3))){
                    return A_DTMF_MAP[c_result][u8_index];
                        
                    }
            }
        return 0x00;
}

int main() {
    stdio_init_all();
    
    // init the ADC
    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);  // ADC0 (GPIO26)
    while (!stdio_usb_connected()) {}
    __uint16_t samples[SAMPLES];

    while (1) {
        // Collect samples
        collectSamples(samples);
        //char c_dtmfRead = dtmfDecode(samples);
        
        // DEBUG: Try to detect a 1kHz signal for debugging
        printf("Magnitude: %f\n", goertzel(SAMPLES, 1000, SAMP_RATE, samples));

        
        //not needed
        sleep_ms(500);
    }

    return 0;
}

