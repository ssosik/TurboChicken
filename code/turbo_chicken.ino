#include <math.h>

//uncomment if you want serial output (measured_voltage  resistance  dutyCycle)
//#define DEBUG

//setting the I/O pins
//the fan PWM output pin
const int fanOutput = 33;
//rotary potentiometer pin
const int rotPot = 32;
const in ledPin = 18;

//settings for the fan curve
//comment if you want the fans to spin at the minimum speed (minDutyCycle) instead of turning off
//#define FAN_OFF
//the minimum PWM duty cycle is when the fan just starts to spin. 40 is about right for most fans. Set it even if you use FAN_OFF as it's used in the fan curve calculation!
const int minDutyCycle = 40;

//maximum fan speed, from 0 to 255 (63 is 25%, 127 is 50%, 190 is 75%)
const int maxDutyCycle = 255;

//select the fan curve you want to use. Only uncomment one!
//#define CURVE_EXP //exponential curve. Use for temperatures from 15 to 30C
//#define CURVE_TIMESTWO //Use for temperatures up to 125C, gentler slope
//#define CURVE_TIMESTHREE //Use for temperatures up to 85C, more aggressive
#define CURVE_SIGMOID  //approximation of a sigmoid curve. It's a ramp that starts climbing from MIN and caps off at MAX, with the fan speed at maxDutyCycle
const int MIN = 2;     //only for the sigmoid curve - described in the README file
const int MAX = 32;    //only for the sigmoid curve - described in the README file

//variables for the resistance calculation
const float V_OFFSET = 0.1;  //offset between the readings and the actual voltage input, because the esp32 ADC is garbage. Documented in the README
const float Vcc = 3.3;       //voltage reference of the voltage divider
const float R_1 = 100000.0;  //resistance in Ohms of the rotary potentiometer
const int averaging = 3;    //how many readings we average to spit out a value, higher is slower but more stable

//initializing our duty cycle variable
int dutyCycle = 0;

//PWM properties (for fans, a 25khz frequency is needed). No need to edit this
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

void setup() {

#ifdef DEBUG
  //initiate the serial connection to tell the values to the computer
  Serial.begin(115200);
  delay(1000);
#endif

  //attaching the ledc channel to the output pin
  if (!ledcAttachChannel(fanOutput, freq, resolution, ledChannel)) {
    Serial.println("LEDC ATTACH CHANNEL FAILED");
  }

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
}

void loop() {
  float voltage = getVoltage(averaging);
#ifdef DEBUG
  float resistance = getResistance(averaging);
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print("   Resistance:");
  Serial.print(resistance / 1000);
  Serial.print("K");
#endif

  //controlling the fan speed with the potentiometer value
  //using a fan curve
#ifdef CURVE_EXP
  dutyCycle = pow(voltage * 10, 2);
#endif
#ifdef CURVE_TIMESTWO
  dutyCycle = voltage * 10 * 2;
#endif
#ifdef CURVE_TIMESTHREE
  dutyCycle = voltage * 10 * 3;
#endif
#ifdef CURVE_SIGMOID
  dutyCycle = (((voltage * 10 - MIN) / (MAX - MIN)) * (maxDutyCycle - minDutyCycle)) + minDutyCycle;
#endif

  //if the duty cycle is under minDutyCycle, set it to minDutyCycle (doesn't turn the fan off)
  if (dutyCycle <= minDutyCycle) {
    dutyCycle = minDutyCycle;
  }
#ifdef FAN_OFF
  //if the temperature is under minTemp, turn off the fan. Only active if FAN_OFF is defined in the settings
  if (voltage * 10 <= MIN) {
    dutyCycle = 0;
  }
#endif

  //cap the duty cycle to MaxDutyCycle
  if (dutyCycle > maxDutyCycle) {
    dutyCycle = maxDutyCycle;
  }

  //check if the PWM dutycycle is inside the bounds
  if (dutyCycle < 0) {
    dutyCycle = 0;
  }
  if (dutyCycle > 255) {
    dutyCycle = 255;
  }

  //send the PWM to the fan, writing on serial for debugging
  if (ledcWrite(fanOutput, dutyCycle)) {
    Serial.println("LEDC WRITE FAILED");
  }

#ifdef DEBUG
  Serial.print("   DutyCycle:");
  Serial.println(dutyCycle);
#endif

  //wait a bit between repeating all the checks
  delay(250);
}

float getVoltage(int avg_size) {
  float V_sum = 0.0;
  for (int i = 0; i < avg_size; i++) {
    //reading the thermistor value
    int potValue = analogRead(rotPot);
    //converting it to a voltage (imperfect, the ESP32 has a non-linear ADC, and summing the values together
    //Serial.print("   potValue:");
    //Serial.println(potValue);
    float potVoltage = ((potValue / 4095.0) * Vcc) + V_OFFSET;
    V_sum += potVoltage;
  }
  //dividing the sum of the readings to get the average
  float V_approx = V_sum / float(avg_size);

  //returning the voltage reading. Because of the offset there could be weird readings at the extremes, that are eliminated
  if (V_approx <= 0.0) {
    return 0.0;
  } else if (V_approx > Vcc) {
    return Vcc;
  } else {  //if(V_approx <= Vcc) and more than 0V, everything is OK
    return V_approx;
  }
}

float getResistance(int avg_size) {
  //getting a voltage reading
  float V_0 = getVoltage(avg_size);
  //converting the voltage into a resistance since R1 is known
  float resistance = (V_0 * R_1) / (Vcc - V_0);

  return resistance;
}
