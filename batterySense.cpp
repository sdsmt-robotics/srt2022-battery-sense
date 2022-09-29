#include "batterySense.h"

SRTBatterySense::SRTBatterySense(int _sensePin,
                                 int _calibrationPinLow = A3,
                                 int _calibrationPinHigh = A6)
{
  sensePin = _sensePin;
  calibrationPinLow = _calibrationPinLow;
  calibrationPinHigh = _calibrationPinHigh;
}


void SRTBatterySense::init()
{
  pinMode(sensePin, INPUT);

  EEPROM.begin(12);
  EEPROM.get(0, adc1);
  EEPROM.get(4, adc2);
  EEPROM.end();

  for (int i = 0; i < BATTERY_CHECK_NUM_SAMPLES; i++)
  {
    batteryCheckSamples[i] = 7.2;
  }
}

void SRTBatterySense::calibrate()
{
  Serial.begin(115200);
  
  Serial.print("Running Calibration...");
  

  // read calibration pins

  int total = 0;

  for (size_t j = 0; j < calibrationReadings; j++)
  {
    total += analogRead(calibrationPinLow);
  }
  adc1 = total/calibrationReadings;
  
  total = 0;

  for (size_t j = 0; j < calibrationReadings; j++)
  {
    total += analogRead(calibrationPinHigh);
  }
  adc2 = total/calibrationReadings;

  
  //save calibration   
  EEPROM.begin(12);

  EEPROM.put(0, adc1);
  EEPROM.put(4, adc2);
  EEPROM.commit();

  EEPROM.end();


  Serial.println("Finished");

  Serial.println(v1);
  Serial.println(adc1);
  Serial.println(v2);
  Serial.println(adc2);  
}

float SRTBatterySense::getBatteryVoltage()
{
  float v = calculateVoltage(sampleADC());
  return v;
  //return adcToBatteryVoltage(sampleADC());
}

uint32_t SRTBatterySense::sampleADC()
{
  uint32_t sum = 0;
  uint32_t avg = 0;
  for (int i = 0; i < NUM_BATTERY_SAMPLES; i++)
  {
    sum += analogRead(sensePin);
  }
  avg = sum / NUM_BATTERY_SAMPLES;

  return avg;
}

//float SRTBatterySense::adcToBatteryVoltage(uint32_t adc)
//{
//  return map(adc, adc1, adc2, v1, v2);
//}


float SRTBatterySense::calculateVoltage(uint32_t rawADC)
{
  float voltage = 0;
  int avgOffset = ((((ADCSteps * (v1/ADCMaxVol)) - adc1) +
                    ((ADCSteps * (v2/ADCMaxVol)) - adc2)) / 2);
  
  //Serial.print("Raw value: ");
  //Serial.println(rawADC);

  rawADC += avgOffset;
  
  //Serial.print("Raw value with offset: ");
  //Serial.println(rawADC);

  voltage = (rawADC / ADCSteps) * ADCMaxVol;
  
  //Serial.print("ADC voltage: ");
  //Serial.println(voltage);
  
  voltage = voltage  * ( 1 / volDivRatio );
  
  //Serial.print("Calculated value: ");
  //Serial.println(voltage);

  return voltage;
}



float SRTBatterySense::getRollingAverage()
{
  if (millis() > batteryCheckPrevTime + BATTERY_CHECK_PERIOD || batteryCheckPrevTime == 0)
  {
    batteryCheckSamples[batteryCheckAverageIndex++] = getBatteryVoltage();
    if (batteryCheckAverageIndex = BATTERY_CHECK_NUM_SAMPLES)
    {
      batteryCheckAverageIndex = 0;
    }

    batteryCheckSum = 0;
    for (int i = 0; i < BATTERY_CHECK_NUM_SAMPLES; i++)
    {
      batteryCheckSum += batteryCheckSamples[i];
    }
    batteryCheckAverage = batteryCheckSum / BATTERY_CHECK_NUM_SAMPLES;

    batteryCheckPrevTime = millis();
  }

  return batteryCheckAverage;
}
