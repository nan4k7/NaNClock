//#define DEBUG //comment below to disable serial in-/output and free some RAM
//#define USEWIFI //enable WiFi support, further down, enter ssid/password there
#define FADING // uncomment to enable fading effects for dots/digits, other parameters further down below
#define TEMPDISPLAY // uncomment this to enable tempDisplay(). It's an example of how to display values at specified times, like temperature readouts
#define PIR // sets a PIR to lower brightness when no movement
#define USERTC

#include <EEPROM.h>
#include <FastLED.h>
#include <RtcDS3231.h>
#include <TimeLib.h>
#include <Wire.h>

#ifdef USEWIFI
	#include <WiFi.h>
	#include <WiFiUdp.h>
#endif


/* Start RTC config/parameters------------------------------------------------------------------------- */
#ifdef USERTC
	RtcDS3231<TwoWire> Rtc(Wire);
#endif
/* End RTC config/parameters---------------------------------------------------------------------------- */

/* Start WiFi config/parameters------------------------------------------------------------------------- */
#ifdef USEWIFI
	const char* wifiSSID = "Fibergarch 2.4";
	const char* wifiPWD = "Hipolito480";
#endif
/* End WiFi config/parameters--------------------------------------------------------------------------- */

/* Start button config/pins----------------------------------------------------------------------------- */
#define BUTTON_A_PIN 3 // momentary push button, 1 pin to gnd, 1 pin GPIO_3
#define BUTTON_B_PIN 4 // momentary push button, 1 pin to gnd, 1 pin GPIO_4
/* End button config/pins------------------------------------------------------------------------------- */

/* Start basic appearance config------------------------------------------------------------------------ */
int8_t dotsBlinking = 1; // 1 = only light up dots on even seconds, 0 = always off, -1 always on
const bool leadingZero = false; // true = enable a leading zero, 9:00 -> 09:00, 1:30 -> 01:30...
uint8_t displayMode = 0; // 0 = 24h mode, 1 = 12h mode ("1" will also override setting that might be written to EEPROM!)
uint8_t colorMode = 0; // different color modes, setting this to anything else than zero will overwrite values written to eeprom, as above
uint16_t colorSpeed = 750; // controls how fast colors change, smaller = faster (interval in ms at which color moves inside colorizeOutput();) orig: 750
const bool reverseColorCycling = false; // true = reverse color movements
const uint8_t brightnessLevels[5] { 2, 45, 115, 190, 255 }; // 0 - 255, brightness Levels (min, med, max) - index will be saved to eeprom
uint8_t brightness = brightnessLevels[0]; // default brightness if none saved to eeprom yet / first run

const uint8_t brightnessNightLevel = 1; // Brillo mínimo para modo nocturno
const uint8_t nightColor[2] = { 0, brightnessNightLevel };

/* Fading options--------------------------------------------------------------------------------------- */
#ifdef FADING
	uint8_t fadeDigits = 2; // fade digit segments, 0 = disabled, 1 = only fade out segments turned off, 2 = fade old out and fade new in
	uint8_t fadeDots = 2; // fade dots, 0 = disabled, 1 = turn dots off without fading in/out after specidfied time, 2 = fade in and out
	uint8_t fadeDelay = 5; // milliseconds between each fading step, 5-25 should work okay-ish orig: 20
#endif
/* End basic appearance config-------------------------------------------------------------------------- */

// Array de horas en las que se activa el modo nocturno (formato 24h, 0-23)
const uint8_t NIGHT_HOURS[] = { 3, 4, 5, 6, 7, 8 }; // Modo nocturno de 4:00 a 7:59
bool isNightMode = false; // Estado actual del modo nocturno (también controla el PIR)

/* Start PIR config/pins----------------------------------------------------------------------------- */
#ifdef PIR
	#define PIR_PIN 1

	bool motionDetected = false;
	uint8_t lastBrightness = brightnessLevels[0];
#endif
/* End PIR config/pins------------------------------------------------------------------------------- */

/* End of basic config/parameters section */

/* Start of FastLED/clock stuff */

#define FASTLED_ESP8266_RAW_PIN_ORDER // this means we'll be using the raw esp8266 pin order -> GPIO_2
#define LED_PIN 2 // led data in connected to GPIO_2

#define LED_PWR_LIMIT 350 // 500mA - Power limit in mA (voltage is set in setup() to 5v)
#define LED_DIGITS 4
#define LED_COUNT 67 // Total number of leds, 67 on S7ripClock - Basic Edition, resulting in leds[0] - leds[66]

uint8_t markerHSV[3] = { 0, 127, 20 }; // this color will be used to "flag" leds for coloring later on while updating the leds
CRGB leds[LED_COUNT];
CRGBPalette16 currentPalette;

// start clock specific config/parameters
/* Segment order, seen from the front:

   <  A  >
 /\       /\
 F        B
 \/       \/
   <  G  >
 /\       /\
 E        C
 \/       \/
   <  D  >

digit positions, seen from the front:
 _   _   _   _   _   _
|_| |_| |_| |_| |_| |_|
|_| |_| |_| |_| |_| |_|

 5   4   3   2   1   0

Note: Digit positions for showSegments() depends on the order in which the segments
are defined in segGroups[] below. Most of my things/clocks published so far start
from the right side when seen from the front, as above. But some have different
orders, like Lazy 7 - QBE, which is using a single strip and has an order of
3, 0, 2, 1 for top left, top right, bottom left, bottom right.

*/

/* Below is the configuration for led <> segment assignments.
	LED_ACCESS_MODE 0 will use the two values inside each segment (led a, led b)
	as they are - 2 leds per segment.
	LED_ACCESS_MODE 1 will use the two values inside each segment (led a, led b)
	as start and end value to get 2+ leds/segment.

	Example:
	leds 0, 3 -> MODE 0 -> led 0 and 3 inside the segment -> 2 leds
	leds 0, 3 -> MODE 1 -> led 0 - 3 inside the segment -> 4 leds

	Simply add all the leds into their corresponding segments inside the array.
	The order of digits/strip routing doesn't really matter there, positions of
	HH:MM:SS are assigned using digitPositions.

	digitsLAM -> LED_ACCESS_MODE per digit
*/

// defining access modes for each digit individually
uint8_t digitsLAM[6] = { 0, 0, 0, 0, 0, 0 };

const uint8_t digitPositions[4] = { 3, 2, 1, 0 }; // positions of HH:MM (3, 0, 2, 1 on L7-QBE)
const uint16_t segGroups[28][2] PROGMEM = {
	/* segments 0-27, 4 digits x 7 segments */
	/* digit position 0 */
	{ 4, 5 }, // top, a
	{ 6, 7 }, // top right, b
	{ 9, 10 }, // bottom right, c
	{ 11, 12 }, // bottom, d
	{ 13, 14 }, // bottom left, e
	{ 2, 3 }, // top left, f
	{ 0, 1 }, // center, g
	/* digit position 1 */
	{ 26, 27 }, // top, a
	{ 28, 29 }, // top right, b
	{ 17, 18 }, // bottom right, c
	{ 19, 20 }, // bottom, d
	{ 21, 22 }, // bottom left, e
	{ 24, 25 }, // top left, f
	{ 30, 31 }, // center, g
	/* digit position 2 */
	{ 39, 40 }, // top, a
	{ 41, 42 }, // top right, b
	{ 44, 45 }, // bottom right, c
	{ 46, 47 }, // bottom, d
	{ 48, 49 }, // bottom left, e
	{ 37, 38 }, // top left, f
	{ 35, 36 }, // center, g
	/* digit position 3 */
	{ 61, 62 }, // top, a
	{ 63, 64 }, // top right, b
	{ 52, 53 }, // bottom right, c
	{ 54, 55 }, // bottom, d
	{ 56, 57 }, // bottom left, e
	{ 59, 60 }, // top left, f
	{ 65, 66 } // center, g
};

const uint16_t upperDots[1] PROGMEM = { 32 }; // leds inside the upper dots
const uint16_t lowerDots[1] PROGMEM = { 34 }; // leds inside the lower dots

// Using above arrays it's very easy to "talk" to the segments. Simply use 0-6 for the first 7 segments, add 7 (7-13)
// for the second one, 14-20 for third....
const uint8_t digits[21][7] PROGMEM = {
	/* Lets define 10 numbers (0-9) with 7 segments each, also adding some letters
	1 = segment is on, 0 = segment is off */
	{ 1, 1, 1, 1, 1, 1, 0 }, // 0 -> Show segments a - f, don't show g (center one)
	{ 0, 1, 1, 0, 0, 0, 0 }, // 1 -> Show segments b + c (top right and bottom right), nothing else
	{ 1, 1, 0, 1, 1, 0, 1 }, // 2 -> and so on...
	{ 1, 1, 1, 1, 0, 0, 1 }, // 3
	{ 0, 1, 1, 0, 0, 1, 1 }, // 4
	{ 1, 0, 1, 1, 0, 1, 1 }, // 5
	{ 1, 0, 1, 1, 1, 1, 1 }, // 6
	{ 1, 1, 1, 0, 0, 0, 0 }, // 7
	{ 1, 1, 1, 1, 1, 1, 1 }, // 8
	{ 1, 1, 1, 1, 0, 1, 1 }, // 9
	{ 0, 0, 0, 1, 1, 1, 1 }, // t -> some letters/symbols from here on (index 10-20, so this won't...
	{ 0, 0, 0, 0, 1, 0, 1 }, // r -> ...interfere with using digits 0-9 by using index 0-9
	{ 0, 1, 1, 1, 0, 1, 1 }, // y
	{ 0, 1, 1, 1, 1, 0, 1 }, // d
	{ 1, 0, 0, 1, 1, 1, 0 }, // C
	{ 1, 0, 0, 0, 1, 1, 1 }, // F
	{ 1, 1, 0, 0, 1, 1, 0 }, // some kind of "half letter M" (left half), displayed using two digits
	{ 1, 1, 1, 0, 0, 1, 0 }, // some kind of "half letter M" (right half), displayed using two digits
	{ 1, 1, 0, 0, 0, 1, 1 }, // °
	{ 0, 1, 1, 0, 1, 1, 1 }, // H
	{ 0, 0, 0, 0, 0, 0, 0 } // "blank"
};

uint8_t clockStatus = 1; // Used for various things, don't mess around with it! 1 = startup, 0 = regular mode, 1 = startup, 9x = setup modes (90, 91, 92, 93...)

/* these values will be saved to EEPROM:
	0 = index for selected palette
	1 = index for selected brightness level
	2 = displayMode, 12h/24h mode
	3 = colorMode */

/* End of FastLED/clock stuff */
// End clock specific configs/parameters

/* other variables */
uint8_t btnRepeatCounter = 0; // keeps track of how often a button press has been repeated
/* */

/* -- this is where the fun parts start
 * -------------------------------------------------------------------------------------------------------- */

#ifdef PIR
	void IRAM_ATTR onMotionChange() {
		if (isNightMode)
			return; // Si estamos en modo nocturno, ignoramos el PIR

		int state = digitalRead(PIR_PIN);

		if (state == HIGH) {
			motionDetected = true;
			Serial.println("PIR detected movement");
			brightness = lastBrightness;
		}
		else {
			motionDetected = false;
			Serial.println("PIR has no movement");
			brightness = brightnessLevels[0];
		}
	}
#endif

void setup() {
	setCpuFrequencyMhz(40);

	Wire.begin();
	Wire.setClock(50000);

	#ifdef DEBUG
		while (millis() < 2000) { // safety delay for serial outputun 
			yield();
		}

		Serial.begin(74880);
		Serial.println(F("  "));

		Serial.print(F("LED power limit: "));
		Serial.print(LED_PWR_LIMIT);
		Serial.println(F(" mA"));
		Serial.print(F("Total LED count: "));
		Serial.println(LED_COUNT);

		#ifdef USEWIFI
			Serial.println(F("WiFi enabled"));
		#endif

		while (millis() < 600) { // safety delay for serial output
			yield();
		}
	#endif

	pinMode(BUTTON_A_PIN, INPUT_PULLUP);
	pinMode(BUTTON_B_PIN, INPUT_PULLUP);

	#ifdef PIR
		pinMode(PIR_PIN, INPUT);
		attachInterrupt(digitalPinToInterrupt(PIR_PIN), onMotionChange, CHANGE);
	#endif

	#ifdef DEBUG
		if (digitalRead(BUTTON_A_PIN) == LOW || digitalRead(BUTTON_B_PIN) == LOW) {
			if (digitalRead(BUTTON_A_PIN) == LOW) {
				Serial.println(F("BUTTON_A_PIN is LOW / pressed - check wiring!"));
			}
			if (digitalRead(BUTTON_B_PIN) == LOW) {
				Serial.println(F("BUTTON_B_PIN is LOW / pressed - check wiring!"));
			}
		}
	#endif

	FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT)
		.setCorrection(TypicalSMD5050)
		.setTemperature(DirectSunlight)
		.setDither(1);
	FastLED.setMaxPowerInVoltsAndMilliamps(5, LED_PWR_LIMIT);
	FastLED.clear();

	#ifdef DEBUG
		Serial.println(F("setup(): Lighting up some leds..."));
	#endif

	for (uint8_t i = 0; i < LED_DIGITS; i++) {
		showSegment(6, i);
	}
	FastLED.show();

	#ifdef USEWIFI // ...and if using WiFi.....
		#ifdef DEBUG
			Serial.println(F("Starting up WiFi..."));
		#endif

		WiFi.mode(WIFI_STA); // set WiFi mode to STA...
		WiFi.begin(wifiSSID, wifiPWD); // ...or credentials defined in the USEWIFI config section

		#ifdef DEBUG
			Serial.println(F("Using credentials from sketch"));
		#endif

		unsigned long startTimer = millis();
		uint8_t wlStatus = 0;
		uint8_t counter = 6;

		#ifdef DEBUG
			Serial.print(F("Waiting for WiFi connection... "));
		#endif

		while (wlStatus == 0) {
			if (WiFi.status() != WL_CONNECTED)
				wlStatus = 0;
			else
				wlStatus = 1;

			if (millis() - startTimer >= 1000) {
				FastLED.clear();
				showDigit(counter, digitPositions[3]);
				FastLED.show();

				if (counter > 0)
					counter--;
				else
					wlStatus = 2;

				startTimer = millis();

				#ifdef DEBUG
					Serial.print(F("."));
				#endif
			}

			yield();
		}

		#ifdef DEBUG
			Serial.println();
			if (WiFi.status() != 0) {
				Serial.print(F("setup(): Connected to SSID: "));
				Serial.println(WiFi.SSID());
			}
			else
				Serial.println(F("setup(): WiFi connection failed."));
		#endif
	#endif

	#ifdef USERTC
		Rtc.Begin();
		if (Rtc.GetIsRunning() == false) {
			#ifdef DEBUG
				Serial.println(F("setup(): RTC not running, trying to start..."));
			#endif

			Rtc.SetIsRunning(true);
		}

		#ifdef DEBUG
			Serial.println(F("setup(): RTC.begin(), 2 second safety delay before"));
			Serial.println(F(" doing any read/write actions!"));
		#endif

		unsigned long tmp_time = millis();
		while (millis() - tmp_time < 2000)
			yield();

		#ifdef DEBUG
			Serial.println(F("setup(): RTC initialized"));
		#endif
	#else
		#ifdef DEBUG
			Serial.println(F("setup(): No RTC defined!"));
			setTime(12, 0, 0, 1, 1, 2000);
		#endif
	#endif

	FastLED.clear();
	FastLED.show();
	/* eeprom settings */
	EEPROM.begin(512);

	paletteSwitcher();
	brightnessSwitcher();
	colorModeSwitcher();
	displayModeSwitcher();

	clockStatus = 0; // change from 1 (startup) to 0 (running mode)

	#ifdef DEBUG
		printTime();
		Serial.println(F("setup() done"));
		Serial.println(F("------------------------------------------------------"));
	#endif
}

/* Night mode functions */
void checkAndUpdateNightMode() {
	#ifdef USERTC
		RtcDateTime now = Rtc.GetDateTime();
		uint8_t currentHour = now.Hour();
	#else
		uint8_t currentHour = hour();
	#endif

	// Comprobamos si la hora actual está en el array de horas nocturnas
	bool shouldBeNightMode = false;
	for (uint8_t i = 0; i < sizeof(NIGHT_HOURS) / sizeof(NIGHT_HOURS[0]); i++) {
		if (currentHour == NIGHT_HOURS[i]) {
			shouldBeNightMode = true;
			break;
		}
	}

	// Si el estado del modo nocturno ha cambiado
	if (shouldBeNightMode != isNightMode) {
		#ifdef DEBUG
			Serial.print(F("cambio estado nightMode: "));

		#endif

		isNightMode = shouldBeNightMode;

		if (isNightMode) {
			Serial.println(F("true"));

			brightness = brightnessNightLevel;
			dotsBlinking = 0;
			fadeDigits = 0;
			fadeDots = 0;
		}
		else {
			Serial.println(F("false"));

			// Saliendo del modo nocturno
			#ifdef PIR
				brightness = motionDetected ? lastBrightness : brightnessLevels[0];
				dotsBlinking = motionDetected ? 1 : 0;
				fadeDigits = motionDetected ? 2 : 0;
				fadeDots = motionDetected ? 2 : 0;
			#else
				brightness = lastBrightness;
				dotsBlinking = 1;
				fadeDigits = 2;
				fadeDots = 2;
			#endif
		}
	}
}

/* MAIN LOOP */
void loop() {
	static uint8_t lastInput = 0; // != 0 if any button press has been detected
	static uint8_t lastSecondDisplayed = 0; // This keeps track of the last second when the display was updated (HH:MM and HH:MM:SS)
	static unsigned long lastCheckRTC = millis(); // This will be used to read system time in case no RTC is defined (not supported!)
	static bool doUpdate = false; // Update led content whenever something sets this to true. Coloring will always happen at fixed intervals!

	#ifdef USERTC
		static RtcDateTime rtcTime = Rtc.GetDateTime(); // Get time from rtc
	#else
		static time_t sysTime = now(); // if no rtc is defined, get local system time
	#endif

	static uint8_t refreshDelay = 75; // refresh leds every orig: 5ms
	static long lastRefresh = millis(); // Keeps track of the last led update/FastLED.show() inside the loop

	if (lastInput != 0) { // If any button press is detected...
		if (btnRepeatCounter < 1) { // execute short/single press function(s)
			#ifdef DEBUG
				Serial.print(F("loop(): "));
				Serial.print(lastInput);
				Serial.println(F(" (short press)"));
			#endif

			if (lastInput == 1) { // short press button A
				brightnessSwitcher();
			}
			if (lastInput == 2) { // short press button B
				paletteSwitcher();
			}
			if (lastInput == 3) { // short press button A + button B
				FastLED.clear();
				FastLED.show();
				setupClock(); // start date/time setup
			}
		}
		else if (btnRepeatCounter > 8) { // execute long press function(s)...
			btnRepeatCounter = 1; // ..reset btnRepeatCounter to stop this from repeating

			#ifdef DEBUG
				Serial.print(F("loop(): "));
				Serial.print(lastInput);
				Serial.println(F(" (long press)"));
			#endif

			if (lastInput == 1) { // long press button A
				colorModeSwitcher();
			}
			if (lastInput == 2) { // long press button B
				displayModeSwitcher();
			}
			if (lastInput == 3) { // long press button A + button B
				esp_restart();
			}
			while (digitalRead(BUTTON_A_PIN) == LOW || digitalRead(BUTTON_B_PIN) == LOW) { // wait until buttons are released again
				if (millis() % 50 == 0) { // Refresh leds every 50ms to give optical feedback
					colorizeOutput(colorMode);
					FastLED.show();
				}

				yield();
			}
		}
	}

	uint8_t refreshWindowMils = 300;

	if (isNightMode) {
		refreshWindowMils = 1000 * 30; //reviso y actualizo cada 30s
		refreshDelay = 1000 * 30; //reviso y actualizo cada 30s
	}


	if (millis() - lastCheckRTC >= refreshWindowMils) { // check rtc/system time every orig: 50ms
		#ifdef USERTC
			rtcTime = Rtc.GetDateTime();
			if (lastSecondDisplayed != rtcTime.Second()) {
				doUpdate = true;
			}
		#else
			sysTime = now();
			if (lastSecondDisplayed != second(sysTime)) {
				doUpdate = true;
			}
		#endif

		lastCheckRTC = millis();
	}

	if (doUpdate) { // this will update the led array if doUpdate is true because of a new second from the rtc
		checkAndUpdateNightMode(); // Verificamos el modo nocturno

		#ifdef USERTC
			setTime(rtcTime.Hour(), rtcTime.Minute(), rtcTime.Second(), rtcTime.Day(), rtcTime.Month(), rtcTime.Year()); // sync system time to rtc every second
			FastLED.clear(); // 1A - clear all leds...
			displayTime(now()); // 2A - output rtcTime to the led array..
			lastSecondDisplayed = rtcTime.Second();
		#else
			FastLED.clear(); // 1B - clear all leds...
			displayTime(now()); // 2B - output sysTime to the led array...
			lastSecondDisplayed = second(sysTime);
		#endif

		#ifdef TEMPDISPLAY
			if (!isNightMode)
				tempDisplay(); // 3AB - if tempDisplay is defined this will clear the led array again to display custom
		#endif

		doUpdate = false;
		#ifdef DEBUG
			if (second() % 20 == 0) {
				printTime();
			}
		#endif
	}

	colorizeOutput(colorMode); // 1C, 2C, 3C...colorize the data inside the led array right now...

	#ifdef FADING
		digitsFader();
		dotsFader();
	#endif

	if (millis() - lastRefresh >= refreshDelay) {
		FastLED.show();
		lastRefresh = millis();
	}

	lastInput = inputButtons();

	//TODO: ACA PODRIA IR UN SLEEP
	
}

#ifdef TEMPDISPLAY
	void tempDisplay() {
		if ((second() >= 20 && second() < 25) || (second() >= 40 && second() < 45)) { // only do something if current second is 30-39
			float rtcTemp = Rtc.GetTemperature().AsFloatDegC(); // get temperature in °C as float (25.75°C)....
			uint8_t tmp = round(rtcTemp); // ...and round (26°C)

			FastLED.clear();
			showDigit(tmp / 10, digitPositions[0]); // tmp (26°C) / 10 = 2 on position 1 of HH
			showDigit(tmp % 10, digitPositions[1]); // tmp (26°C) % 10 = 6 on position 2 of HH
			showDigit(18, digitPositions[2]); // ° symbol from array digits[][] on position 1 of MM
			showDigit(14, digitPositions[3]); // C from array digits[][] on position 2 of MM
		}
	}
#endif

#ifdef FADING
	void fadeSegment(uint8_t pos, uint8_t segment, uint8_t amount, uint8_t fadeType) {
		/* this will check if the first led of a given segment is lit and if it is, will fade by
		amount using fadeType. fadeType is important because when fading things in that where
		off previously we must avoid setting them black at first - hence fadeLightBy instead
		of fadeToBlack. */
		uint8_t ledAM = digitsLAM[pos]; // led access mode according to the position
		if (leds[pgm_read_word_near(&segGroups[segment + pos * 7][0])]) {
			if (ledAM == 0) {
				for (uint8_t i = 0; i < 2; i++) {
					if (fadeType == 0) {
						leds[pgm_read_word_near(&segGroups[segment + pos * 7][i])].fadeToBlackBy(amount);
					}
					else {
						leds[pgm_read_word_near(&segGroups[segment + pos * 7][i])].fadeLightBy(amount);
					}
				}
			}

			if (ledAM == 1) {
				uint16_t startLed = pgm_read_word_near(&segGroups[segment + pos * 7][0]);
				uint16_t endLed = pgm_read_word_near(&segGroups[segment + pos * 7][1]);
				for (uint16_t i = startLed; i <= endLed; i++) {
					if (fadeType == 0) {
						leds[i].fadeToBlackBy(amount);
					}
					else {
						leds[i].fadeLightBy(amount);
					}
				}
			}
		}
	}

	void digitsFader() {
		if (fadeDigits == 0)
			return;

		static unsigned long firstRun = 0; // time when a change has been detected and fading starts
		static unsigned long lastRun = 0; // used to store time when this function was executed the last time
		static boolean active = false; // will be used as a flag when to do something / fade segments
		static uint8_t previousSegments[LED_DIGITS][7] = { 0 }; // all the segments lit after the last run
		static uint8_t currentSegments[LED_DIGITS][7] = { 0 }; // all the segments lit right now
		static uint8_t changedSegments[LED_DIGITS][7] = { 0 }; // used to store the differences -> 1 = led has been turned off, fade out, 2 = was off, fade in
		static uint8_t fadeSteps = 12; // steps used to fade dots in or out orig: 15
		lastRun = millis();

		if (!active) { // this will check if....
			firstRun = millis();
			for (uint8_t digitPos = 0; digitPos < LED_DIGITS; digitPos++) { // ...any of the segments are on....
				for (uint8_t segmentPos = 0; segmentPos < 7; segmentPos++) {
					if (leds[pgm_read_word_near(&segGroups[segmentPos + digitPos * 7][0])]) {
						currentSegments[digitPos][segmentPos] = 1;
					}
					else {
						currentSegments[digitPos][segmentPos] = 0;
					}

					if (currentSegments[digitPos][segmentPos]
						!= previousSegments[digitPos]
										[segmentPos]) { // ...and compare them to the previous displayed segments.
						active = true; // if a change has been detected, set active = true so fading gets executed

						#ifdef DEBUG
							Serial.print(F("digitPos: "));
							Serial.print(digitPos);
							Serial.print(F(" - segmentPos: "));
							Serial.print(segmentPos);
							Serial.print(F(" was "));
						#endif

						if (currentSegments[digitPos][segmentPos] == 0) {
							changedSegments[digitPos][segmentPos] = 1;

							#ifdef DEBUG
								Serial.println(F("ON, is now OFF"));
							#endif
						}
						else {
							changedSegments[digitPos][segmentPos] = 2;

							#ifdef DEBUG
								Serial.println(F("OFF, is now ON"));
							#endif
						}
					}
				}
			}
		}

		if (active) { // this part is executed once a change has been detected....
			static uint8_t counter = 1;
			static unsigned long lastFadeStep = millis();
			for (uint8_t digitPos = 0; digitPos < LED_DIGITS;
				digitPos++) { // redraw segments that have turned off, so we can fade them out...
				for (uint8_t segmentPos = 0; segmentPos < 7; segmentPos++) {
					if (changedSegments[digitPos][segmentPos] == 1) {
						showSegment(segmentPos, digitPos);
					}
				}
			}
			colorizeOutput(colorMode); // colorize again after redraw, so colors keep consistent
			for (uint8_t digitPos = 0; digitPos < LED_DIGITS; digitPos++) {
				for (uint8_t segmentPos = 0; segmentPos < 7; segmentPos++) {
					if (changedSegments[digitPos][segmentPos]
						== 1) { // 1 - segment has turned on, this one has to be faded in
						fadeSegment(digitPos, segmentPos, counter * (255.0 / fadeSteps),
							0); // fadeToBlackBy, segments supposed to be off/fading out
					}
					if (changedSegments[digitPos][segmentPos]
						== 2) { // 2 - segment has turned off, this one has to be faded out
						if (fadeDigits == 2) {
							fadeSegment(digitPos, segmentPos, 255 - counter * (255.0 / fadeSteps),
								1); // fadeLightBy, segments supposed to be on/fading in
						}
					}
				}
			}

			if (millis() - lastFadeStep >= fadeDelay) {
				counter++;
				lastFadeStep = millis();
			}

			if (counter > fadeSteps) { // done with fading, reset variables...
				counter = 1;
				active = false;
				for (uint8_t digitPos = 0; digitPos < LED_DIGITS;
					digitPos++) { // and save current segments to previousSegments
					for (uint8_t segmentPos = 0; segmentPos < 7; segmentPos++) {
						if (leds[pgm_read_word_near(&segGroups[segmentPos + digitPos * 7][0])])
							previousSegments[digitPos][segmentPos] = 1;
						else
							previousSegments[digitPos][segmentPos] = 0;

						changedSegments[digitPos][segmentPos] = 0;
					}
				}

				#ifdef DEBUG
					Serial.print(F("digit fading sequence took ")); // for debugging/checking duration - fading should never
																	// take longer than 1000ms!
					Serial.print(millis() - firstRun);
					Serial.println(F(" ms"));
				#endif
			}
		}
	}

	void dotsFader() {
		if (fadeDots == 0)
			return;

		static unsigned long firstRun = 0;
		static unsigned long lastRun = 0;
		static boolean active = false;
		static uint8_t fadeSteps = 15;
		lastRun = millis();

		if (!active) {
			if (leds[pgm_read_word_near(&upperDots[0])]) {
				active = true;
				firstRun = millis();
			}
		}

		if (fadeDots == 1 && active) { // action = 1, simply turn off specidifc leds after 500ms
			if (lastRun - firstRun >= 500) {
				for (uint8_t i = 0; i < (sizeof(upperDots) / sizeof(upperDots[0])); i++) {
					leds[pgm_read_word_near(&upperDots[i])].setHSV(0, 0, 0);
				}
				for (uint8_t i = 0; i < (sizeof(lowerDots) / sizeof(lowerDots[0])); i++) {
					leds[pgm_read_word_near(&lowerDots[i])].setHSV(0, 0, 0);
				}
				active = false;
			}
		}

		if (fadeDots == 2 && active) { // fade in/out dots
			static uint8_t counter = 1;
			static unsigned long lastFadeStep = millis();
			static boolean fadeInDone = true;
			if (!fadeInDone) {
				for (uint8_t i = 0; i < (sizeof(upperDots) / sizeof(upperDots[0])); i++) {
					leds[pgm_read_word_near(&upperDots[i])].fadeToBlackBy(255 - counter * (255.0 / fadeSteps));
				}
				for (uint8_t i = 0; i < (sizeof(lowerDots) / sizeof(lowerDots[0])); i++) {
					leds[pgm_read_word_near(&lowerDots[i])].fadeToBlackBy(255 - counter * (255.0 / fadeSteps));
				}
				if (millis() - lastFadeStep >= fadeDelay) {
					counter++;
					lastFadeStep = millis();
				}
				if (counter > fadeSteps) {
					counter = 1;
					fadeInDone = true;
					/*
					#ifdef DEBUG
							Serial.print(F("dot fade-in sequence took ")); // for debugging/checking
							Serial.print(millis() - firstRun);
							Serial.println(F(" ms"));
					#endif
					*/
				}
			}
			if (lastRun - firstRun >= 950 - fadeDelay * fadeSteps) {
				for (uint8_t i = 0; i < (sizeof(upperDots) / sizeof(upperDots[0])); i++) {
					leds[pgm_read_word_near(&upperDots[i])].fadeToBlackBy(counter * (255.0 / fadeSteps));
				}
				for (uint8_t i = 0; i < (sizeof(lowerDots) / sizeof(lowerDots[0])); i++) {
					leds[pgm_read_word_near(&lowerDots[i])].fadeToBlackBy(counter * (255.0 / fadeSteps));
				}
				if (millis() - lastFadeStep >= fadeDelay) {
					counter++;
					lastFadeStep = millis();
				}
				if (counter > fadeSteps) {
					counter = 1;
					active = false;
					fadeInDone = false;
					/*
					#ifdef DEBUG
							Serial.print(F("dot fading sequence took ")); // for debugging/checking
							Serial.print(millis() - firstRun);
							Serial.println(F(" ms"));
					#endif
					*/
				}
			}
		}
	}
#endif

void setupClock() {
	clockStatus = 90; // clockStatus 9x = setup, relevant for other functions/coloring
	while (digitalRead(BUTTON_A_PIN) == LOW || digitalRead(BUTTON_B_PIN) == LOW) { // do nothing until both buttons are released to avoid accidental inputs right away
		yield();
	}

	tmElements_t setupTime; // Create a time element which will be used. Using the current time would
	setupTime.Hour = 12; // give some problems (like time still running while setting hours/minutes)
	setupTime.Minute = 0; // Setup starts at 12 (12 pm)
	setupTime.Second = 0; //
	setupTime.Day = 1;
	setupTime.Month = 2; // see above
	setupTime.Year = 23; // current year - 2000 (2023 - 2000 = 23)

	#ifdef USERTC
		RtcDateTime writeTime;
	#endif

	uint8_t lastInput = 0;
	// hours
	while (lastInput != 2) {
		clockStatus = 92; // 92 = HH setup
		if (lastInput == 1) {
			if (setupTime.Hour < 23)
				setupTime.Hour++;
			else
				setupTime.Hour = 0;
		}
		displayTime(makeTime(setupTime));
		lastInput = inputButtons();
	}
	lastInput = 0;
	// minutes
	while (lastInput != 2) {
		clockStatus = 93; // 93 = MM setup
		if (lastInput == 1) {
			if (setupTime.Minute < 59) {
				setupTime.Minute++;
			} else {
				setupTime.Minute = 0;
			}
		}
		displayTime(makeTime(setupTime));
		lastInput = inputButtons();
	}
	lastInput = 0;

	#ifdef DEBUG
		Serial.print(F("setupClock(): "));
		Serial.print(F("HH:MM:SS -> "));

		if (setupTime.Hour < 10)
			Serial.print(F("0"));
		Serial.print(setupTime.Hour);
		Serial.print(F(":"));
		if (setupTime.Minute < 10)
			Serial.print(F("0"));
		Serial.print(setupTime.Minute);
		Serial.print(F(":"));
		if (setupTime.Second < 10)
			Serial.print(F("0"));
		Serial.println(setupTime.Second);
	#endif

	#ifdef USERTC
		writeTime = { 2000 + setupTime.Year, setupTime.Month, setupTime.Day, setupTime.Hour, setupTime.Minute, setupTime.Second };

		Rtc.SetDateTime(writeTime);
		setTime(
			writeTime.Hour(), writeTime.Minute(), writeTime.Second(), writeTime.Day(), writeTime.Month(), writeTime.Year());

		#ifdef DEBUG
			Serial.println(F("setupClock(): RTC time set"));
			printTime();
		#endif
	#else
		setupTime.Year += 30;
		setTime(makeTime(setupTime));
	#endif
	clockStatus = 0;

	#ifdef DEBUG
		Serial.println(F("setupClock() done"));
	#endif
}

uint16_t getUserInput(uint8_t sym1, uint8_t sym2, uint8_t startVal, uint8_t endVal) {
	/* This will show two symbols on HH and allow to enter a 2 digit value using the buttons
	and display the value on MM. */
	static uint8_t lastInput = 0;
	static uint8_t currentVal = startVal;
	static bool newInput = true;
	if (newInput) {
		currentVal = startVal;
		newInput = false;
	}

	while (lastInput != 2) {
		if (lastInput == 1) {
			if (currentVal < endVal) {
				currentVal++;
			} else {
				currentVal = startVal;
			}
		}

		FastLED.clear();
		showDigit(sym1, digitPositions[0]);
		showDigit(sym2, digitPositions[1]);
		showDigit(currentVal / 10, digitPositions[2]);
		showDigit(currentVal % 10, digitPositions[3]);

		if (millis() % 30 == 0) {
			colorizeOutput(colorMode);
			FastLED.show();
		}

		lastInput = inputButtons();
	}

	#ifdef DEBUG
		Serial.print(F("getUserInput(): returned "));
		Serial.println(currentVal);
	#endif
		lastInput = 0;
		newInput = true;
		return currentVal;
	#ifdef DEBUG
		Serial.print(F("getUserInput(): returned "));
		Serial.println(currentVal);
	#endif
}

void colorizeOutput(uint8_t mode) {
	/* So far showDigit()/showSegment() only set some leds inside the array to values from "markerHSV" but we haven't
	updated the leds yet using FastLED.show(). This function does the coloring of the right now single colored but
	"insivible" output. This way color updates/cycles aren't tied to updating display contents */
	static unsigned long lastColorChange = 0;
	static uint8_t startColor = 0;
	static uint8_t colorOffset = 0; // different offsets result in quite different results, depending on the amount of
									// leds inside each segment...
									// ...so it's set inside each color mode if required
	/* mode 0 = check every segment if it's lit and assign a color based on position -> different color per digit
	Checking the leds like this will not include the dots - they'll be colored later on */
	if (mode == 0) {
		colorOffset = 256 / LED_DIGITS;
		for (uint8_t pos = 0; pos < LED_DIGITS; pos++) {
			for (uint8_t segment = 0; segment < 7; segment++) {
				colorizeSegment(segment, pos, startColor + colorOffset * pos);
			}
		}
	}
	/* mode 1 = simply assign different colors with an offset of "colorOffset" to each led that's not black/off -
	This will include the dots if they're supposed to be on - but they will be overwritten later for all modes */
	if (mode == 1) {
		colorOffset = 32;
		for (uint16_t i = 0; i < LED_COUNT; i++) {
			if (leds[i]) {
				leds[i] = ColorFromPalette(currentPalette, startColor + i * colorOffset, brightness, LINEARBLEND);
			}
		}
	}
	/* mode 2 = check every segment if it's lit and assign a color based on segment -> different color per segment,
	same across digits. Checking the leds like this will not include the dots - they'll be colored later on */
	if (mode == 2) {
		colorOffset = 24;
		for (uint8_t pos = 0; pos < LED_DIGITS; pos++) {
			for (uint8_t segment = 0; segment < 7; segment++) {
				colorizeSegment(segment, digitPositions[pos], startColor + 1 * colorOffset * segment);
			}
		}
	}
	/* mode 3 = same as above - but will assign colorOffsets depending on segment in a specific order (top/down effect)
	*/
	if (mode == 3) {
		uint8_t colorOffsets[7] = { 0, 24, 72, 96, 72, 24, 48 }; // colorOffsets for segments a-g
		for (uint8_t pos = 0; pos < LED_DIGITS; pos++) {
			for (uint8_t segment = 0; segment < 7; segment++) {
				colorizeSegment(segment, digitPositions[pos], startColor + 1 * colorOffsets[segment]);
			}
		}
	}
	/* clockStatus >= 90 is used for coloring output while in setup mode */
	if (clockStatus >= 90) {
		static boolean blinkFlag = true;
		static unsigned long lastBlink = millis();
		static uint8_t b = brightnessLevels[0];
		if (millis() - lastBlink > 333) { // blink switch frequency, 3 times a second
			if (blinkFlag) {
				blinkFlag = false;
				b = brightnessLevels[4];
			}
			else {
				blinkFlag = true;
				b = brightnessLevels[2];
			}

			lastBlink = millis();
		} // unset values = red, set value = green, current value = yellow and blinkinkg
		for (uint8_t pos = 0; pos < LED_DIGITS; pos++) {
			if (clockStatus == 91) { // Y/M/D setup
				colorHelper(digitPositions[0], 0, 255, brightness);
				colorHelper(digitPositions[1], 0, 255, brightness);
				colorHelper(digitPositions[2], 64, 255, b);
				colorHelper(digitPositions[3], 64, 255, b);
			}
			if (clockStatus == 92) { // hours
				colorHelper(digitPositions[0], 64, 255, b);
				colorHelper(digitPositions[1], 64, 255, b);
				colorHelper(digitPositions[2], 0, 255, brightness);
				colorHelper(digitPositions[3], 0, 255, brightness);
			}
			if (clockStatus == 93) { // minutes
				colorHelper(digitPositions[0], 96, 255, brightness);
				colorHelper(digitPositions[1], 96, 255, brightness);
				colorHelper(digitPositions[2], 64, 255, b);
				colorHelper(digitPositions[3], 64, 255, b);
			}
			if (clockStatus == 94) { // seconds
				colorHelper(digitPositions[0], 96, 255, brightness);
				colorHelper(digitPositions[1], 96, 255, brightness);
				colorHelper(digitPositions[2], 96, 255, brightness);
				colorHelper(digitPositions[3], 96, 255, brightness);
			}
		}
	}
	/* The dots will always be colored in the same way, just using colors from the current palette. Depending on
	setup/parameters this can otherwise lead to the dots looking quite different from the digits, so as before
	they're cycling through the color palette once per minute */
	if (leds[pgm_read_word_near(&upperDots[0])]) { // if the first led inside the array upperDot is lit...
		for (uint8_t i = 0; i < (sizeof(upperDots) / sizeof(upperDots[0]));
			i++) { // ...start applying colors to all leds inside the array
			if (clockStatus == 0) {
				leds[pgm_read_word_near(&upperDots[i])] = ColorFromPalette(currentPalette, second() * 4.25, brightness, LINEARBLEND);
			}
			else {
				leds[pgm_read_word_near(&upperDots[i])].setHSV(64, 255, brightness);
			}
		}
	}
	if (leds[pgm_read_word_near(&lowerDots[0])]) { // same as before for the lower dots...
		for (uint8_t i = (sizeof(lowerDots) / sizeof(lowerDots[0])); i > 0; i--) {
			if (clockStatus == 0) {
				leds[pgm_read_word_near(&lowerDots[i - 1])] = ColorFromPalette(currentPalette, second() * 4.25, brightness, LINEARBLEND);
			}
			else {
				leds[pgm_read_word_near(&lowerDots[i - 1])].setHSV(64, 255, brightness);
			}
		}
	}

	if (millis() - lastColorChange > colorSpeed) {
		if (reverseColorCycling) {
			startColor--;
		} else {
			startColor++;
		}
		lastColorChange = millis();
	}

	if (isNightMode && clockStatus == 0) {                           // nightmode will overwrite everything that has happened so far...
		for ( uint16_t i = 0; i < LED_COUNT; i++ ) {
			if ( leds[i] ) {
				leds[i].setHSV(nightColor[0], 255, nightColor[1] );      // and assign nightColor to all lit leds. Default is a very dark red.
				FastLED.setDither(0);
			}
		}
	}
	else {
		FastLED.setDither(1);
	}

}

void colorizeSegment(uint8_t segment, uint8_t pos, uint8_t color) {
	/* Checks if segment at position is on - and if it is, assigns color from current palette */
	uint8_t ledAM = digitsLAM[pos]; // led access mode according to the position
	if (leds[pgm_read_word_near(&segGroups[segment + digitPositions[pos] * 7][0])]) {
		if (ledAM == 0) {
			for (uint8_t i = 0; i < 2; i++) {
				leds[pgm_read_word_near(&segGroups[segment + digitPositions[pos] * 7][i])] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND);
			}
		}
		if (ledAM == 1) {
			uint16_t startLed = pgm_read_word_near(&segGroups[segment + digitPositions[pos] * 7][0]);
			uint16_t endLed = pgm_read_word_near(&segGroups[segment + digitPositions[pos] * 7][1]);
			for (uint16_t i = startLed; i <= endLed; i++) {
				leds[i] = ColorFromPalette(currentPalette, color, brightness, LINEARBLEND);
			}
		}
	}
}

void colorHelper(uint8_t pos, uint8_t hue, uint8_t sat, uint8_t bri) {
	/* Used for coloring digits inside setup routines/steps
	It will simply set the digit at the given position to the given hsv values */
	uint8_t ledAM = digitsLAM[pos]; // led access mode according to the position
	for (uint8_t segment = 0; segment < 7; segment++) {
		if (leds[pgm_read_word_near(&segGroups[segment + pos * 7][0])]) { // if first led inside segment is lit...
			if (ledAM == 0) {
				for (uint8_t i = 0; i < 2; i++) { // assign hue to led 0 + 1 inside segment
					leds[pgm_read_word_near(&segGroups[segment + pos * 7][i])].setHSV(hue, sat, bri);
				}
			}
			if (ledAM == 1) {
				uint16_t startLed = pgm_read_word_near(&segGroups[segment + pos * 7][0]);
				uint16_t endLed = pgm_read_word_near(&segGroups[segment + pos * 7][1]);
				for (uint16_t i = startLed; i <= endLed; i++) { // assign hue to led 0 - 1 inside segment
					leds[i].setHSV(hue, sat, bri);
				}
			}
		}
	}
}

void displayTime(time_t t) {
	if (clockStatus >= 90) {
		FastLED.clear();
	}

	/* hours */
	if (displayMode == 0) {
		if (hour(t) < 10) {
			if (leadingZero) {
				showDigit(0, digitPositions[0]);
			}
		}
		else {
			showDigit(hour(t) / 10, digitPositions[0]);
		}
		showDigit(hour(t) % 10, digitPositions[1]);
	}
	else if (displayMode == 1) {
		if (hourFormat12(t) < 10) {
			if (leadingZero) {
				showDigit(0, digitPositions[0]);
			}
		}
		else {
			showDigit(hourFormat12(t) / 10, digitPositions[0]);
		}
		showDigit(hourFormat12(t) % 10, digitPositions[1]);
	}
	/* minutes */
	showDigit(minute(t) / 10, digitPositions[2]);
	showDigit(minute(t) % 10, digitPositions[3]);

	if (clockStatus >= 90) { // in setup modes displayTime will also use colorizeOutput/FastLED.show!
		static unsigned long lastRefresh = millis();
		if (isAM(t)
			&& displayMode == 1) { // in 12h mode and if it's AM only light up the upper dots (while setting time)
			showDots(1);
		}
		else {
			showDots(2);
		}

		if (millis() - lastRefresh >= 25) {
			colorizeOutput(colorMode);
			FastLED.show();
			lastRefresh = millis();
		}
		return;
	}
	/* dots */
	if (dotsBlinking == 1) {
		if (second(t) % 2 == 0) {
			showDots(2);
		}
	}
	else if (dotsBlinking == -1) {
		showDots(2);
	}
}

void showSegment(uint8_t segment, uint8_t segDisplay) {
	// This shows the segments from top of the sketch on a given position (segDisplay). Order of positions/segDisplay is
	// the order of definitions on the top, first one defined is segDisplay 0, second one is segDisplay 1 and so on...
	// "firstLoop" is used to display all information only once per test if customHelper is defined
	uint8_t ledAM = digitsLAM[segDisplay]; // led access mode according to the position

	if (ledAM == 0) { // using both values inside the array to light up two leds
		segment += segDisplay * 7;
		for (uint8_t i = 0; i < 2; i++) {
			leds[pgm_read_word_near(&segGroups[segment][i])].setHSV(markerHSV[0], markerHSV[1], markerHSV[2]);
		}
	}

	if (ledAM == 1) { // using both values inside the array as start and end to light up multiple leds
		segment += segDisplay * 7;
		uint16_t startLed = pgm_read_word_near(&segGroups[segment][0]);
		uint16_t endLed = pgm_read_word_near(&segGroups[segment][1]);

		for (uint16_t i = startLed; i <= endLed; i++) {
			leds[i].setHSV(markerHSV[0], markerHSV[1], markerHSV[2]);
		}
	}
}

void showDots(int8_t dots) {
	// dots 0 = upper dots, dots 1 = lower dots, dots 2 = all dots (right/left/both on Lazy 7 - Quick Build Edition)

	if (dots == 1 || dots == 2) {
		for (uint8_t i = 0; i < (sizeof(upperDots) / sizeof(upperDots[0])); i++) {
			leds[pgm_read_word_near(&upperDots[i])].setHSV(markerHSV[0], markerHSV[1], markerHSV[2]);
		}
	}
	if (dots == 0 || dots == 2) {
		for (uint8_t i = 0; i < (sizeof(lowerDots) / sizeof(lowerDots[0])); i++) {
			leds[pgm_read_word_near(&lowerDots[i])].setHSV(markerHSV[0], markerHSV[1], markerHSV[2]);
		}
	}
}

void showDigit(uint8_t digit, uint8_t pos) {
	// This draws numbers using the according segments as defined on top of the sketch (0 - 9) or symbols/characters
	// (index 10+)
	for (uint8_t i = 0; i < 7; i++) {
		if (pgm_read_byte_near(&digits[digit][i]) != 0)
			showSegment(i, pos);
	}
}

void paletteSwitcher() {
	/* As the name suggests this takes care of switching palettes. When adding palettes, make sure paletteCount
	increases accordingly. A few examples of gradients/solid colors by using RGB values or HTML Color Codes below */
	static uint8_t paletteCount = 28;
	static uint8_t currentIndex = 0;
	if (clockStatus == 1) { // Clock is starting up, so load selected palette from eeprom...
		uint8_t tmp = EEPROM.read(0);
		if (tmp >= 0 && tmp < paletteCount) {
			currentIndex = tmp; // 255 from eeprom would mean there's nothing been written yet, so checking range...
		}
		else {
			currentIndex = 0; // ...and default to 0 if returned value from eeprom is not 0 - 6
		}

		#ifdef DEBUG
			Serial.print(F("paletteSwitcher(): loaded EEPROM value "));
			Serial.println(tmp);
		#endif
	}

	switch (currentIndex) {
		case 0:
			currentPalette = CRGBPalette16(CRGB::Red);
			break;
		case 1:
			currentPalette = CRGBPalette16(CRGB::Green);
			break;
		case 2:
			currentPalette = CRGBPalette16(CRGB::Blue);
			break;
		case 3:
			currentPalette = CRGBPalette16(CRGB::Yellow);
			break;
		case 4:
			currentPalette = CRGBPalette16(CRGB::Cyan);
			break;
		case 5:
			currentPalette = CRGBPalette16(CRGB::Magenta);
			break;
		case 6:
			currentPalette = CRGBPalette16(CRGB::White);
			break;
		case 7:
			currentPalette = CRGBPalette16(CRGB::DarkOrange);
			break;
		case 8:
			currentPalette = CRGBPalette16(CRGB::Purple);
			break;
		case 9:
			currentPalette = CRGBPalette16(CRGB::HotPink);
			break;
		case 10:
			currentPalette = CRGBPalette16(CRGB::Aqua);
			break;
		case 11:
			currentPalette = CRGBPalette16(CRGB::BlueViolet);
			break;
		case 12:
			currentPalette = CRGBPalette16(CRGB(224, 0, 32), CRGB(0, 0, 244), CRGB(128, 0, 128), CRGB(224, 0, 64));
			break;
		case 13:
			currentPalette = CRGBPalette16(CRGB(224, 16, 0), CRGB(192, 64, 0), CRGB(192, 128, 0), CRGB(240, 40, 0));
			break;
		case 14:
			currentPalette = CRGBPalette16(CRGB::Aquamarine, CRGB::Turquoise, CRGB::Blue, CRGB::DeepSkyBlue);
			break;
		case 15:
			currentPalette = RainbowColors_p;
			break;
		case 16:
			currentPalette = PartyColors_p;
			break;
		case 17:
			currentPalette = CRGBPalette16(CRGB::Gold, CRGB::OrangeRed, CRGB::Tomato, CRGB::FireBrick);
			break;
		case 18:
			currentPalette = CRGBPalette16(CRGB::Teal, CRGB::Cyan, CRGB::DodgerBlue, CRGB::Navy);
			break;
		case 19:
			currentPalette = CRGBPalette16(CRGB::Green, CRGB::ForestGreen, CRGB::SeaGreen, CRGB::MediumSpringGreen);
			break;
		case 20:
			currentPalette = CRGBPalette16(CRGB::Red, CRGB::Crimson, CRGB::DarkRed, CRGB::OrangeRed);
			break;
		case 21:
			currentPalette = CRGBPalette16(CRGB::Yellow, CRGB::Gold, CRGB::Orange, CRGB::SaddleBrown);
			break;
		case 22:
			currentPalette = CRGBPalette16(CRGB::SkyBlue, CRGB::RoyalBlue, CRGB::BlueViolet, CRGB::DarkSlateBlue);
			break;
		case 23:
			currentPalette = CRGBPalette16(CRGB::Purple, CRGB::MediumVioletRed, CRGB::DeepPink, CRGB::HotPink);
			break;
		case 24:
			currentPalette = LavaColors_p;
			break;
		case 25:
			currentPalette = CRGBPalette16(CRGB::Orange, CRGB::Coral, CRGB::Salmon, CRGB::LightCoral);
			break;
		case 26:
			currentPalette = CRGBPalette16(CRGB::MediumPurple, CRGB::SlateBlue, CRGB::Indigo, CRGB::DarkMagenta);
			break;
		case 27:
			currentPalette = CRGBPalette16(CRGB::Chartreuse, CRGB::Lime, CRGB::GreenYellow, CRGB::SpringGreen);
			break;

		}

	#ifdef DEBUG
		Serial.print(F("paletteSwitcher(): selected palette "));
		Serial.println(currentIndex);
	#endif

	if (clockStatus == 0) { // only save selected palette to eeprom if clock is in normal running mode, not while in
							// startup/setup/whatever
		EEPROM.put(0, currentIndex);
		EEPROM.commit();

		#ifdef DEBUG
				Serial.print(F("paletteSwitcher(): saved index "));
				Serial.print(currentIndex);
				Serial.println(F(" to eeprom"));
		#endif
	}

	if (currentIndex < paletteCount - 1) {
		currentIndex++;
	}
	else {
		currentIndex = 0;
	}

	#ifdef DEBUG
		Serial.println(F("paletteSwitcher() done"));
	#endif
}

void brightnessSwitcher() {
	static uint8_t currentIndex = 0;
	static uint8_t brightnessLevelsCount = 5;

	if (clockStatus == 1) { // Clock is starting up, so load selected palette from eeprom...
		uint8_t tmp = EEPROM.read(1);
		if (tmp >= 0 && tmp < brightnessLevelsCount) {
			currentIndex = tmp; // 255 from eeprom would mean there's nothing been written yet, so checking range...
		}
		else {
			currentIndex = 0; // ...and default to 0 if returned value from eeprom is not 0 - 4
		}

		#ifdef DEBUG
			Serial.print(F("brightnessSwitcher(): loaded EEPROM value "));
			Serial.println(tmp);
		#endif
	}

	brightness = brightnessLevels[currentIndex];
	lastBrightness = brightness;

	#ifdef DEBUG
		Serial.print(F("brightnessSwitcher(): selected brightness index "));
		Serial.println(currentIndex);
	#endif

	if (clockStatus == 0) { // only save selected brightness to eeprom if clock is in normal running mode, not while in
							// startup/setup/whatever
		EEPROM.put(1, currentIndex);
		EEPROM.commit();

		#ifdef DEBUG
				Serial.print(F("brightnessSwitcher(): saved index "));
				Serial.print(currentIndex);
				Serial.println(F(" to eeprom"));
		#endif
	}

	if (currentIndex < (brightnessLevelsCount - 1)) {
		currentIndex = (currentIndex + 1) % brightnessLevelsCount;
	}
	else {
		currentIndex = 0;
	}

	#ifdef DEBUG
		Serial.println(F("brightnessSwitcher() done"));
	#endif
}

void colorModeSwitcher() {
	static uint8_t currentIndex = 0;
	if (clockStatus == 1) { // Clock is starting up, so load selected palette from eeprom...
		if (colorMode != 0)
			return; // 0 is default, if it's different on startup the config is set differently, so exit here

		uint8_t tmp = EEPROM.read(3);

		if (tmp >= 0 && tmp < 4) { // make sure tmp < 3 is increased if color modes are added in colorizeOutput()!
			currentIndex = tmp; // 255 from eeprom would mean there's nothing been written yet, so checking range...
		}
		else {
			currentIndex = 0; // ...and default to 0 if returned value from eeprom is not 0 - 2
		}

		#ifdef DEBUG
			Serial.print(F("colorModeSwitcher(): loaded EEPROM value "));
			Serial.println(tmp);
		#endif
	}
	colorMode = currentIndex;

	#ifdef DEBUG
		Serial.print(F("colorModeSwitcher(): selected colorMode "));
		Serial.println(currentIndex);
	#endif

	if (clockStatus == 0) { // only save selected colorMode to eeprom if clock is in normal running mode, not while in
							// startup/setup/whatever
		EEPROM.put(3, currentIndex);
		EEPROM.commit();

		#ifdef DEBUG
			Serial.print(F("colorModeSwitcher(): saved index "));
			Serial.print(currentIndex);
			Serial.println(F(" to eeprom"));
		#endif
	}

	if (currentIndex < 3) {
		currentIndex++;
	}
	else {
		currentIndex = 0;
	}

	#ifdef DEBUG
		Serial.println(F("colorModeSwitcher() done"));
	#endif
}

void displayModeSwitcher() {
	static uint8_t currentIndex = 0;

	if (clockStatus == 1) { // Clock is starting up, so load selected palette from eeprom...
		if (displayMode != 0)
			return; // 0 is default, if it's different on startup the config is set differently, so exit here

		uint8_t tmp = EEPROM.read(2);

		if (tmp >= 0 && tmp < 2) { // make sure tmp < 2 is increased if display modes are added
			currentIndex = tmp; // 255 from eeprom would mean there's nothing been written yet, so checking range...
		}
		else {
			currentIndex = 0; // ...and default to 0 if returned value from eeprom is not 0 - 1 (24h/12h mode)
		}

		#ifdef DEBUG
			Serial.print(F("displayModeSwitcher(): loaded EEPROM value "));
			Serial.println(tmp);
		#endif
	}

	displayMode = currentIndex;

	#ifdef DEBUG
		Serial.print(F("displayModeSwitcher(): selected displayMode "));
		Serial.println(currentIndex);
	#endif

	if (clockStatus == 0) { // only save selected colorMode to eeprom if clock is in normal running mode, not while in
							// startup/setup/whatever
		EEPROM.put(2, currentIndex);
		EEPROM.commit();

		#ifdef DEBUG
			Serial.print(F("displayModeSwitcher(): saved index "));
			Serial.print(currentIndex);
			Serial.println(F(" to eeprom"));
		#endif
	}
	if (clockStatus == 0) { // show 12h/24h for 2 seconds after selected in normal run mode, don't show this on startup (status 1)
		FastLED.clear();
		unsigned long timer = millis();

		while (millis() - timer <= 2000) {
			if (currentIndex == 0) {
				showDigit(2, digitPositions[0]);
				showDigit(4, digitPositions[1]);
				showDigit(19, digitPositions[3]);
			}
			if (currentIndex == 1) {
				showDigit(1, digitPositions[0]);
				showDigit(2, digitPositions[1]);
				showDigit(19, digitPositions[3]);
			}
			colorizeOutput(colorMode);
			if (millis() % 50 == 0) {
				FastLED.show();
			}

			yield();
		}
	}

	if (currentIndex < 1) {
		currentIndex++;
	}
	else {
		currentIndex = 0;
	}

	#ifdef DEBUG
		Serial.println(F("displayModeSwitcher() done"));
	#endif
}

bool leapYear(uint16_t y) {
	boolean isLeapYear = false;
	if (y % 4 == 0)
		isLeapYear = true;
	if (y % 100 == 0 && y % 400 != 0)
		isLeapYear = false;
	if (y % 400 == 0)
		isLeapYear = true;
	if (isLeapYear)
		return true;
	else
		return false;
}

uint8_t inputButtons() {
	/* This scans for button presses and keeps track of delay/repeat for user inputs
		Short keypresses will only be returned when buttons are released before repeatDelay
		is reached. This is to avoid constantly sending 1 or 2 when executing a long button
		press and/or multiple buttons.
		Note: Buttons are using pinMode INPUT_PULLUP, so HIGH = not pressed, LOW = pressed! */
	static uint8_t scanInterval = 30; // only check buttons every 30ms
	static uint16_t repeatDelay = 1000; // delay in milliseconds before repeating detected keypresses
	static uint8_t repeatRate = 1000 / 10; // 10 chars per 1000 milliseconds
	static uint8_t minTime = scanInterval * 2; // minimum time to register a button as pressed
	static unsigned long lastReadout = millis(); // keeps track of when the last readout happened
	static unsigned long lastReturn = millis(); // keeps track of when the last readout value was returned
	static uint8_t lastState = 0; // button state from previous scan
	uint8_t currentState = 0; // button state from current scan
	uint8_t retVal = 0; // return value, will be 0 if no button is pressed
	static unsigned long eventStart = millis(); // keep track of when button states are changing

	if (millis() - lastReadout < scanInterval)
		return 0; // only scan for button presses every <scanInterval> ms
	if (digitalRead(BUTTON_A_PIN) == LOW)
		currentState += 1;
	if (digitalRead(BUTTON_B_PIN) == LOW)
		currentState += 2;
	if (currentState == 0 && currentState == lastState) {
		btnRepeatCounter = 0;
	}

	if (currentState != 0 && currentState != lastState) { // if any button is pressed and different from the previous scan...
		eventStart = millis(); // ...reset eventStart to current time
		btnRepeatCounter = 0; // ...and reset global variable btnRepeatCounter
	}

	if (currentState != 0 && currentState == lastState) { // if same input has been detected at least twice (2x scanInterval)...
		if (millis() - eventStart >= repeatDelay) { // ...and longer than repeatDelay...
			if (millis() - lastReturn >= repeatRate) { // ...check for repeatRate...
				retVal = currentState; // ...and set retVal to currentState
				btnRepeatCounter++;
				lastReturn = millis();
			} else
				retVal = 0; // return 0 if repeatDelay hasn't been reached yet
		}
	}

	if (currentState == 0 && currentState != lastState && millis() - eventStart >= minTime && btnRepeatCounter == 0) {
		retVal = lastState; // return lastState if all buttons are released after having been pressed for <minTime> ms
		btnRepeatCounter = 0;
	}

	lastState = currentState;
	lastReadout = millis();

	#ifdef DEBUG // output some information and read serial input, if available
		if (retVal != 0) {
			Serial.print(F("inputButtons(): Return value is: "));
			Serial.print(retVal);
			Serial.print(F(" - btnRepeatCounter is: "));
			Serial.println(btnRepeatCounter);
		}
	#endif

	return retVal;
}

// functions below will only be included if DEBUG is defined on top of the sketch
#ifdef DEBUG
	void printTime() {
		/* outputs current system and RTC time to the serial monitor */
		time_t tmp = now();

		#ifdef USERTC
			RtcDateTime tmp2 = Rtc.GetDateTime();
			setTime(tmp2.Hour(), tmp2.Minute(), tmp2.Second(), tmp2.Day(), tmp2.Month(), tmp2.Year());
			tmp = now();
		#endif

		Serial.println(F("-----------------------------------"));
		Serial.print(F("System time is : "));
		if (hour(tmp) < 10)
			Serial.print(F("0"));

		Serial.print(hour(tmp));
		Serial.print(F(":"));

		if (minute(tmp) < 10)
			Serial.print(F("0"));

		Serial.print(minute(tmp));
		Serial.print(F(":"));

		if (second(tmp) < 10)
			Serial.print(F("0"));

		Serial.println(second(tmp));
		Serial.print(F("System date is : "));
		Serial.print(year(tmp));
		Serial.print("-");
		Serial.print(month(tmp));
		Serial.print("-");
		Serial.print(day(tmp));
		Serial.println(F(" (Y/M/D)"));

		#ifdef USERTC
			Serial.print(F("RTC time is: "));

			if (tmp2.Hour() < 10)
				Serial.print(F("0"));

			Serial.print(tmp2.Hour());
			Serial.print(F(":"));

			if (tmp2.Minute() < 10)
				Serial.print(F("0"));

			Serial.print(tmp2.Minute());
			Serial.print(F(":"));

			if (tmp2.Second() < 10)
				Serial.print(F("0"));

			Serial.println(tmp2.Second());
			Serial.print(F("RTC date is: "));
			Serial.print(tmp2.Year());
			Serial.print("-");
			Serial.print(tmp2.Month());
			Serial.print("-");
			Serial.print(tmp2.Day());
			Serial.println(F(" (Y/M/D)"));
		#endif

		Serial.println(F("-----------------------------------"));
	}
#endif