#include <LiquidCrystal.h>

// FHT, http://wiki.openmusiclabs.com/wiki/ArduinoFHT
#define LOG_OUT 1 // use the log output function
#define LIN_OUT8 1 // use the linear byte output function
#define FHT_N 256 // set to 256 point fht
#include <FHT.h> // include the library

// pins
#define MicPin A0 // used with analogRead mode only

// consts
#define AmpMax (1024 / 2)
#define MicSamples (1024*2) // Three of these time-weightings have been internationally standardised, 'S' (1 s) originally called Slow, 'F' (125 ms) originally called Fast and 'I' (35 ms) originally called Impulse.

// modes
#define Use3.3 // use 3.3 voltage. the 5v voltage from usb is not regulated. this is much more stable.
#define ADCReClock // switch to higher clock, not needed if we are ok with freq between 0 and 4Khz.
#define ADCFlow // read data from adc with free-run (not interupt). much better data, dc low. hardcoded for A0.

#define FreqLog // use log scale for FHT frequencies
#ifdef FreqLog
#define FreqOutData fht_log_out
#define FreqGainFactorBits 0
#else
#define FreqOutData fht_lin_out8
#define FreqGainFactorBits 3
#endif
#define FreqSerialBinary

#define VolumeGainFactorBits 0

// macros
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// LCD
LiquidCrystal lcd(9, 8, 7, 6, 5, 4);
char lcdLineBuf[16 + 1];

void setup()
{
	//pinMode(MicPin, INPUT); // relevant for digital pins. not relevant for analog. however, don't put into digital OUTPUT mode if going to read analog values.

#ifdef ADCFlow
	// set the adc to free running mode
	// register explanation: http://maxembedded.com/2011/06/the-adc-of-the-avr/
	// 5 => div 32. sample rate 38.4
	// 7 => switch to divider=128, default 9.6khz sampling
	ADCSRA = 0xe0+7; // "ADC Enable", "ADC Start Conversion", "ADC Auto Trigger Enable" and divider.
	ADMUX = 0x0; // use adc0 (hardcoded, doesn't use MicPin). Use ARef pin for analog reference (same as analogReference(EXTERNAL)).
#ifndef Use3.3
	ADMUX |= 0x40; // Use Vcc for analog reference.
#endif
	DIDR0 = 0x01; // turn off the digital input for adc0
#else
#ifdef Use3.3
	analogReference(EXTERNAL); // 3.3V to AREF
#endif
#endif

#ifdef ADCReClock // change ADC freq divider. default is div 128 9.6khz (bits 111)
	// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
	// 1 0 0 = mode 4 = divider 16 = 76.8khz
	//sbi(ADCSRA, ADPS2);
	//cbi(ADCSRA, ADPS1);
	//cbi(ADCSRA, ADPS0);
	// 1 0 1 = mode 5 = divider 32 = 38.4Khz
	sbi(ADCSRA, ADPS2);
	cbi(ADCSRA, ADPS1);
	sbi(ADCSRA, ADPS0);
#endif

	// serial
	Serial.begin(9600);
	while (!Serial); // Wait untilSerial is ready - Leonardo
	Serial.println("Starting mic demo");

	// lcd
	lcd.begin(16, 2);  // set up the LCD's number of columns and rows: 
	lcd.clear();  //Clears the LCD screen and positions the cursor in the upper-left corner.   
	lcd.setCursor(0, 0);                    // set the cursor to column 6, line 0
	lcd.print("Sound: ");
}

void loop()
{
	// what do we want to do?
	//MeasureAnalog();
	MeasureVolume();
	//MeasureFHT();
}

// measure basic properties of the input signal
// determine if analog or digital, determine range and average.
void MeasureAnalog()
{
	long signalAvg = 0, signalMax = 0, signalMin = 1024, t0 = millis();
	//cli();  // UDRE interrupt slows this way down on arduino1.0
	for (int i = 0; i < MicSamples; i++)
	{
#ifdef ADCFlow
		while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
		sbi(ADCSRA, ADIF); // restart adc
		byte m = ADCL; // fetch adc data
		byte j = ADCH;
		int k = ((int)j << 8) | m; // form into an int
#else
		int k = analogRead(MicPin);
#endif
		signalMin = min(signalMin, k);
		signalMax = max(signalMax, k);
		signalAvg += k;
	}
	signalAvg /= MicSamples;
	//sei();

	// print
	Serial.print("Time: " + String(millis() - t0));
	Serial.print(" Min: " + String(signalMin));
	Serial.print(" Max: " + String(signalMax));
	Serial.print(" Avg: " + String(signalAvg));
	Serial.print(" Span: " + String(signalMax - signalMin));
	Serial.print(", " + String(signalMax - signalAvg));
	Serial.print(", " + String(signalAvg - signalMin));
	Serial.println("");

	//lcd.clear();  //Clears the LCD screen and positions the cursor in the upper-left corner.   
	//lcd.setCursor(6, 0);                    // set the cursor to column 0, line 0
	//sprintf(lcdLineBuf, "%3d%% %3ddB", (int)soundVolRMS, (int)dB);
	//lcd.print(lcdLineBuf);
}

// calculate volume level of the signal and print to serial and LCD
void MeasureVolume()
{
	long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = millis();
	//cli();  // UDRE interrupt slows this way down on arduino1.0
	for (int i = 0; i < MicSamples; i++)
	{
#ifdef ADCFlow
		while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
		sbi(ADCSRA, ADIF); // restart adc
		byte m = ADCL; // fetch adc data
		byte j = ADCH;
		int k = ((int)j << 8) | m; // form into an int
#else
		int k = analogRead(MicPin);
#endif
		int amp = abs(k - AmpMax);
		amp <<= VolumeGainFactorBits;
		soundVolMax = max(soundVolMax, amp);
		soundVolAvg += amp;
		soundVolRMS += ((long)amp*amp);
	}
	soundVolAvg /= MicSamples;
	soundVolRMS /= MicSamples;
	float soundVolRMSflt = sqrt(soundVolRMS);
	//sei();

	float dB = 20.0*log10(soundVolRMSflt/AmpMax);

	// convert from 0 to 100
	soundVolAvg = 100 * soundVolAvg / AmpMax; 
	soundVolMax = 100 * soundVolMax / AmpMax; 
	soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
	soundVolRMS = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin)

	// print
	Serial.print("Time: " + String(millis() - t0));
	Serial.print(" Amp: Max: " + String(soundVolMax));
	Serial.print("% Avg: " + String(soundVolAvg));
	Serial.print("% RMS: " + String(soundVolRMS));
	Serial.println("% dB: " + String(dB,3));

	//lcd.clear();  //Clears the LCD screen and positions the cursor in the upper-left corner.   
	lcd.setCursor(6, 0);                    // set the cursor to column 6, line 0
	sprintf(lcdLineBuf, "%3d%% %3ddB", (int)soundVolRMS, (int)dB);
	lcd.print(lcdLineBuf);

}

// calculate frequencies in the signal and print to serial
void MeasureFHT()
{
	long t0 = micros();
#ifdef ADCFlow
	//cli();  // UDRE interrupt slows this way down on arduino1.0
#endif
	for (int i = 0; i < FHT_N; i++) { // save 256 samples
#ifdef ADCFlow
		while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
		sbi(ADCSRA, ADIF); // restart adc
		byte m = ADCL; // fetch adc data
		byte j = ADCH;
		int k = ((int)j << 8) | m; // form into an int
#else
		int k = analogRead(MicPin);
#endif
		k -= 0x0200; // form into a signed int
		k <<= 6; // form into a 16b signed int
		k <<= FreqGainFactorBits;
		fht_input[i] = k; // put real data into bins
	}
#ifdef ADCFlow
	//sei();
#endif
	long dt = micros() - t0;
	fht_window(); // window the data for better frequency response
	fht_reorder(); // reorder the data before doing the fht
	fht_run(); // process the data in the fht
#ifdef FreqLog
	fht_mag_log();
#else
	fht_mag_lin8(); // take the output of the fht
#endif

#ifdef FreqSerialBinary
	// print as binary
	Serial.write(255); // send a start byte
	Serial.write(FreqOutData, FHT_N / 2); // send out the data
#else
	// print as text
	for (int i = 0; i < FHT_N / 2; i++)
	{
		Serial.print(FreqOutData[i]);
		Serial.print(',');
	}
	long sample_rate = FHT_N * 1000000l / dt;
	Serial.print(dt);
	Serial.print(',');
	Serial.println(sample_rate);
#endif
}
