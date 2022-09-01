/******************************************************************************
AuxTalon
Interface for Aux Talon
Bobby Schulz @ GEMS Sensing
5/18/2022
https://github.com/gemsiot/Driver_-_Talon-Auxiliary

Allows control of all elements of the Aux Talon, including IO interfacing and self diagnostics 

0.0.0

///////////////////////////////////////////////////////////////////FILL QUOTE////////////////////////////////////////////////////////////////////////////////

Distributed as-is; no warranty is given.
******************************************************************************/
/**
 * @file AuxTalon.h
 *
 * @mainpage Auxiliary Talon
 *
 * @section description Description
 * An interface to exist between the Auxiliary Talon and the Kestrel data logger
 *
 * @section circuit Circuit
 * - Red LED connected to pin D2.
 * - Momentary push button connected to pin D3.
 *
 * @section libraries Libraries
 * - Arduino_LSM6DS3 (https://github.com/arduino-libraries/Arduino_LSM6DS3)
 *   - Interacts with on-board IMU.
 *
 * @section notes Notes
 * - Comments are Doxygen compatible.
 *
 * @section todo TODO
 * - Don't use Doxygen style formatting inside the body of a function.
 *
 * @section author Author
 * - Created by Bobby Schulz on 5/20/2022
 *
 * Copyright (c) 2020 Woolsey Workshop.  All rights reserved.
 */

#ifndef AuxTalon_h
#define AuxTalon_h

#include <stdio.h>
#include "Arduino.h"
#include <Wire.h>
#include "PCAL9535A.h"
#include <Sensor.h>
#include <Talon.h>
#include <GlobalPins.h>
// #include "Adafruit_ADS1015.h"

// enum GroupMode{
//  Dim = 0,
//  Blink = 1,
// };

// enum OutputMode{
//  OpenDrain = 0,
//  TotemPole = 1,
// };

// namespace OutputType 
// {
//     constexpr uint8_t OPEN_DRAIN = 1;
//     constexpr uint8_t PUSH_PULL = 0;
// };

//! Pins for IO expander Alpha
/*! Variable names corespond directly to signal names in Eagle CAD. */
namespace pinsAlpha 
{
  constexpr uint8_t REG_EN = 7; ///< OUTPUT: Pin to enable the 5V regulator. HIGH = ON, LOW = OFF
  constexpr uint8_t ADC_INT = 12; ///< INPUT: Interrupt output from ADS1115 ADC. Active LOW
  constexpr uint8_t RST = 6;
  constexpr uint8_t EN1 = 5;
  constexpr uint8_t EN2 = 3;
  constexpr uint8_t EN3 = 1;
  constexpr uint8_t FAULT1 = 0;
  constexpr uint8_t FAULT2 = 2;
  constexpr uint8_t FAULT3 = 4;
  constexpr uint8_t MUX_SEL0 = 8;
  constexpr uint8_t MUX_SEL1 = 9;
  constexpr uint8_t MUX_SEL2 = 10;
  constexpr uint8_t MUX_EN = 11;
  constexpr uint8_t ACTRL1 = 13;
  constexpr uint8_t ACTRL2 = 14;
  constexpr uint8_t ACTRL3 = 15;
};

namespace pinsBeta 
{
  constexpr uint8_t OUT1 = 0;
  constexpr uint8_t OUT2 = 1;
  constexpr uint8_t OUT3 = 2;
  constexpr uint8_t COUNT_EN1 = 3;
  constexpr uint8_t COUNT_EN2 = 4;
  constexpr uint8_t COUNT_EN3 = 5;
  constexpr uint8_t LOAD = 6;
  constexpr uint8_t OVF1 = 7;
  constexpr uint8_t OVF2 = 8;
  constexpr uint8_t OVF3 = 9;
  constexpr uint8_t D1_SENSE = 10;
  constexpr uint8_t D2_SENSE = 11;
  constexpr uint8_t D3_SENSE = 12;
  constexpr uint8_t OD1 = 13;
  constexpr uint8_t OD2 = 14;
  constexpr uint8_t OD3 = 15;
};

/**
 * @class AuxTalon
 * @brief Library for the Auxiliar Talon interfacing
 */
class AuxTalon: public Talon
{
  constexpr static  int DEAFULT_PORT = 4; ///<Use port 4 by default
  constexpr static  int DEFAULT_VERSION = 0x14; ///<Use hardware version v1.4 by default
  constexpr static  int MAX_NUM_ERRORS = 10; ///<Maximum number of errors to log before overwriting previous errors in buffer
  const String FIRMWARE_VERSION = "1.2.0"; //FIX! Read from system??
  
  ////////////// ERROR CODES ///////////////
  const uint32_t AUX_ADC_READ_FAIL = 0x100E0000; ///<Can't talk to ADC over I2C
  const uint32_t ADC_TIMEOUT = 0x80070000; ///<Communication works, but new reading not returned in time
  const uint32_t COUNTER_OVERFLOW = 0xF00F0000; ///<Counter for a given port has overflowed 
  const uint32_t TIME_DELTA_EXCEEDED = 0xF0100000; ///<Time is non-zero, but delta between start and stop is unreasonable
  const uint32_t TIME_BAD = 0xF0110000; ///<Time given is non-sensical
  const uint32_t DEVICE_RESET = 0xF0120000; ///<Device has been reset since the last read
  const uint32_t AUX_POWER_FAIL = 0x20020000; ///<Failure in sensor power detected
  const uint32_t AUX_POWER_FAIL_PERSISTENT = 0x20020100; ///<Power failure continues even after power cycle
  const uint32_t BUS_DISAGREE = 0x70020000; ///<Input and output measure of bus are outside of valid range
  const uint32_t BUS_OUTOFRANGE = 0x20030000; ///<Bus is outside of specified range
  const uint32_t IO_INIT_FAIL = 0x100F0000; ///<Failure to initialize IO expander, port coresponds to which IO expander (1 = Alpha, 2 = Beta, 3 = Gamma)
  const uint32_t AUX_ADC_INIT_FAIL = 0x10100000; ///<Failure to initialize ADC
  const uint32_t INPUT_BUFF_FAIL = 0x30010000; ///<Input buffer does not pass signal
  const uint32_t COUNTER_INCREMENT_FAIL = 0x80060000; ///<Failure to increment the counter, but buffer works
  const uint32_t COUNTER_CLEAR_FAIL = 0x80050000; ///<Counter fails to clear when ordered - low 2 bytes are Talon and port
  const uint32_t TALON_EEPROM_READ_FAIL = 0x10090000; ///<Report failure to read from Talon EEPROM 
  // const uint32_t TALON_PORT_RANGE_ERROR = 0x90010200; ///<Talon port assignment is out of range 
  const float MAX_DISAGREE = 0.1; //If bus is different from expected by more than 10%, throw error

  

  public:

    /**
     * @brief Instantiate the Talon, defaults to using pre-specified port and hardware version
     * * @param[in] talonPort: coresponds with the number of the port on the Kestrel logger that the Talon is plugged into
     * * @param[in] version: Describes the hardware version of the Talon to be used 
     * @details Serves only to copy set global values of port and version
     */    
    AuxTalon(uint8_t talonPort_ = DEAFULT_PORT, uint8_t version = DEFAULT_VERSION); 
    /**
     * @brief Setsup the Talon for operation and sets all configurations
     * @details Configures IO expander pins, runs a level 2 Self Diagnostic, and resets the counters 
     */ 
    String begin(time_t time, bool &criticalFault, bool &fault);
    int sleep(bool State);

    uint16_t counts[3] = {0}; //Used to store the register values after a read 
    float rates[3] = {0}; //Used to store the calculated rate values from registers
    bool overflow[3] = {false}; //Used to store if any of the registers have overflowed in the previous period
    bool faults[3] = {false}; //Used to store if any of the ports have had a power fault 
    float analogValsAvg[3] = {0}; //Used to store the averaged analog reading outputs
    float analogVals[3] = {0}; //Used to store the instantanious analog values
    bool portVoltageSettings[3] = {0}; //Used to store the configurations of each port (0 = 3v3, 1 = 5v0) - updated at each call of level 4 or greater
    
    // String errorTags[MAX_NUM_ERRORS]
    
    String getData(time_t time);
    int restart();
    String selfDiagnostic(uint8_t diagnosticLevel = 4, time_t time = 0); //Default to just level 4 diagnostic, default to time = 0
    // int sleepMode(uint8_t mode) //DEFINE!
    // int reportErrors(uint32_t *errors, size_t length);
    String getErrors();
    String getMetadata();
    // uint8_t totalErrors();
    // bool ovfErrors();
    // uint8_t getPort();
    // void setTalonPort(uint8_t port_);
    // bool isTalon() {
    //   return true;
    // };
    int enableData(uint8_t port, bool state);
    int enablePower(uint8_t port, bool state);
    int disableDataAll();
    int disablePowerAll();
    // uint8_t getTalonPort() {
    //   return talonPort + 1;
    // }
    bool isPresent();
    uint8_t getNumPorts() {
      return numPorts;
    }

    // const uint8_t sensorInterface = BusType::NONE;
    bool automaticGainControl = true; ///<Flag to configure the automatic gain control for analog sensing (Default = true)
    uint8_t samplesToAverage = 128; ///<Flag to configure the number of samples to average across for analog sensing (Default = 128)
   
  
  private:
    // Adafruit_ADS1115 ads(0x49); 
    const int ADR_ADS1115 = 0x49;
    const unsigned long adcReadTimeout = 10; //Wait at most 10ms for an updated reading
    const time_t maxTimeDelta = 0; //FIX!!! 
    const uint8_t numPorts = 3; 
    PCAL9535A ioAlpha; //ADR = 0x20
    PCAL9535A ioBeta; //ADR = 0x23
    PCAL9535A ioGamma; //ADR = 0x24

    

    // int throwError(uint32_t error);

    int16_t adcRead(uint8_t port, uint8_t gain); 
    int adcConfig(uint8_t configHigh, uint8_t configLow);
    int16_t readADCReg(uint8_t reg);
    int16_t readADCReg(); //Re-reads the previous address
    void setPinDefaults();


    int clearCount(time_t time); 
    int readCounters();

    bool hasReset(); 

    /**
     * @brief Grabs data from all counters and resets them
     * * @param[in] time: Current logger time, used to create a timebase for rate values 
     * @details if time is not provided or is invalid, then calls for count will work as expected, but rate calls will return an error
     */ 
    int updateCount(time_t time);
    int updateCount(); //Have data updated without updating timebase 
    /**
     * @brief Grabs data from all analog input channels
     * * @param[in] samplesToAverage: the desired number (0 to 255) of samples to perform a rolling average over. Defaults to 0.
     * * @param[in] automaticGainControl: specifies if automatic gain control is used. If true, the gain of the ADC will be adjusted automatically based on the range of input value  
     * @details records N samples in a rolling average and stores the values for collection later
     */ 
    int updateAnalog();
    /**
     * @brief Returns the average rate of pulses  
     * * @param[in] port: The port to collect the values for
     * * @return average rate of pulses [pulses/second] over the previous interval 
     * @details calculated over the period between the last clearCount(time_t time) call and the last updateCount(time_t time). If no valid timebase is present, will return -9999.0
     */ 
    float getRate(uint8_t port);
    /**
     * @brief Returns the number of pulses
     * * @param[in] port: The port to collect the values for
     * * @return total number of pulses over the previous interval
     * @details interval is between the last clearCount(time_t time) call and the last updateCount(time_t time)
     */ 
    uint16_t getCount(uint8_t port);

    


    time_t clearTime = 0;
    // static time_t readTime = 0;
    // bool initDone = false; //Used to keep track if the initaliztion has run - used by hasReset() 
    

    // uint32_t errors[MAX_NUM_ERRORS] = {0};
    // uint8_t numErrors = 0; //Used to track the index of errors array
    // bool errorOverwrite = false; //Used to track if errors have been overwritten in time since last report
    // bool timeBaseGood = false; //Used to keep track of the valitity of the current timebase
    // uint8_t talonPort = 0; //Used to keep track of which port the Talon is connected to on Kestrel
    // uint32_t portErrorCode = 0; //Used to easily OR with error codes to add the Talon port
    uint8_t version = 0; //FIX! This should be read from EEPROM in future 
    //////// ADC CONFIG VALS //////////////
    const uint8_t adcBaseConfigHigh = 0x01; //Single shot, blanked port and gain
    const uint8_t adcBaseConfigLow = 0x80; //128 sps
    const uint8_t adcGainConfigs[6] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x10}; //6.144V, 4.096V, 2.048V, 1.024V, 0.512V, 0.256V
    const float adcGainConv[6] = {0.1875, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125}; //Multiply output by conversion value to get mV
    const uint8_t adcPortConfigs[4] = {0x40, 0x50, 0x60, 0x70}; //Port0, Port1, Port2, Port3
    const uint8_t adcStartConversion = 0x80; 
    uint32_t portErrorCode = 0; //Used to easily OR with error codes to add the Talon port
};

#endif