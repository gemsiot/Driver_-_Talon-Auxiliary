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

#include <AuxTalon.h>

AuxTalon::AuxTalon(uint8_t talonPort, uint8_t version) : ioAlpha(0x20), ioBeta(0x23), ioGamma(0x24)
{
	
}

String AuxTalon::begin(time_t time, bool &criticalFault, bool &fault) 
{
	//Only use isEnabled() if using particle
	#if defined(ARDUINO) && ARDUINO >= 100 
		Wire.begin();
	#elif defined(PARTICLE)
		if(!Wire.isEnabled()) Wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
	#endif

	// bool criticalFault = false; //Used to keep track if a critical error has been encountered during the initialization
	// bool fault = false; //Used to keep track if a non-critical error has been encountered during the initialization
	int startingErrors = numErrors; //Grab the number of errors which have been logged when we start the begin call, used to keep track of new errors
	//Initalize io expanders 
	int ioError[3] = {0};
	ioError[0] = ioAlpha.begin();
	ioError[1] = ioBeta.begin();
	ioError[2] = ioGamma.begin();	

	for(int i = 0; i < 3; i++) {
		if(ioError[i] != 0) { 
			throwError(IO_INIT_ERROR | ioError[i]); //Throw error on first init error, not again 
			criticalFault = true; //If any IO expander fails, this is a critical error  
			break;
		}
	}

	Wire.beginTransmission(ADR_ADS1115);
	Wire.write(0x00);
	if(Wire.endTransmission() != 0) {
		throwError(ADC_INIT_ERROR); //Throw ADC initialization error
		fault = true; //Set non-critical fault flag
	}
	// ads.begin();

	setPinDefaults();
	
	///////////////////// RUN DIAGNOSTICS /////////////
	String diagnosticResults = selfDiagnostic(2); //Run level two diagnostic

	////////// RESET COUNTERS //////////////////////////
	clearCount(time); //Clear counter and pass time info in
	// ioAlpha.digitalWrite(pinsAlpha::RST, LOW);
	// ioAlpha.digitalWrite(pinsAlpha::RST, HIGH); //Reset counters
	// ioBeta.digitalWrite(pinsBeta::LOAD, HIGH); //Load new counter values //TEST: check delta in fall-rise time to make sure there is enough time between reset and load 
	// ioBeta.digitalWrite(pinsBeta::LOAD, LOW);




	initDone = true; //Set init flag
	// if(criticalFault == true) return -1; //If a critical fault was detected, return with critical fault code
	if(numErrors - startingErrors > 0 || fault == true) fault = true; //If a non-critical fault was detected, or additional errors thrown, set fault
	// else return 0; //Only if no additional errors present, return operational state
	return diagnosticResults; //Return diagnostic string
}


// int AuxTalon::reportErrors(uint32_t *errorOutput, size_t length)
// {
// 	if(numErrors > length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, but array provided still too small
// 		for(int i = 0; i < length; i++) { //Write as many as we can back
// 			errorOutput[i] = error[i];
// 		}
// 		return -1; //Throw error for insufficnet array length
// 	}
// 	else if(numErrors < length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, provided array of good size (DESIRED)
// 		for(int i = 0; i < numErrors; i++) { //Write all back into array 
// 			errorOutput[i] = error[i];
// 		}
// 		return 0; //Return success indication
// 	}
// 	else if(numErrors > MAX_NUM_ERRORS && MAX_NUM_ERRORS < length) { //Overwritten, but array of good size 
// 		for(int i = 0; i < MAX_NUM_ERRORS; i++) { //Write all back into array 
// 			errorOutput[i] = error[i];
// 		}
// 		return 1; //Return overwrite indication
// 	}
// 	return -1; //Return fault if unknown cause 
// }
int AuxTalon::sleep(bool State)
{
	return 0; //DEBUG!
}

String AuxTalon::getErrors()
{
	// if(numErrors > length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, but array provided still too small
	// 	for(int i = 0; i < length; i++) { //Write as many as we can back
	// 		errorOutput[i] = error[i];
	// 	}
	// 	return -1; //Throw error for insufficnet array length
	// }
	// if(numErrors < length && numErrors < MAX_NUM_ERRORS) { //Not overwritten, provided array of good size (DESIRED)
	// 	for(int i = 0; i < numErrors; i++) { //Write all back into array 
	// 		errorOutput[i] = error[i];
	// 	}
	// 	return 0; //Return success indication
	// }
	String output = "{\"ERRORS\":{"; // OPEN JSON BLOB
	output = output + "\"CODES\":["; //Open codes pair

	for(int i = 0; i < min(MAX_NUM_ERRORS, numErrors); i++) { //Interate over used element of array without exceeding bounds
		output = output + String(errors[i]) + ","; //Add each error code
		errors[i] = 0; //Clear errors as they are read
	}
	if(output.substring(output.length() - 1).equals(",")) {
		output = output.substring(0, output.length() - 1); //Trim trailing ','
	}
	output = output + "],"; //close codes pair
	output =  output + "\"OW\":"; //Open state pair
	if(numErrors > MAX_NUM_ERRORS) output = output + "1,"; //If overwritten, indicate the overwrite is true
	else output = output + "0,"; //Otherwise set it as clear
	output = output + "\"NUM\":" + String(numErrors); //Append number of errors
	output = output + "}}"; //CLOSE JSON BLOB
	numErrors = 0; //Clear error count
	return output;

	// return -1; //Return fault if unknown cause 
}

int AuxTalon::throwError(uint32_t error)
{
	errors[(numErrors++) % MAX_NUM_ERRORS] = error; //Write error to the specified location in the error array
	if(numErrors > MAX_NUM_ERRORS) errorOverwrite = true; //Set flag if looping over previous errors 
	return numErrors;
}

String AuxTalon::selfDiagnostic(uint8_t diagnosticLevel)
{
	if(diagnosticLevel == 0) {
		//TBD
		return "{\"lvl-0\":{}}";
	}

	else if(diagnosticLevel == 1) {
		//TBD
		return "{\"lvl-1\":{}}";
	}

	else if(diagnosticLevel == 2) {
		//TBD
		return "{\"lvl-2\":{}}";
	}


	else if(diagnosticLevel == 4) {
		// String output = selfDiagnostic(5); //Call the lower level of self diagnostic 
		// output = output.substring(0,output.length() - 1); //Trim off closing brace
		String output = "{\"lvl-4\":{"; //OPEN JSON BLOB

		ioAlpha.digitalWrite(pinsAlpha::EN1, HIGH); //Make sure all ports are enabled before testing 
		ioAlpha.digitalWrite(pinsAlpha::EN2, HIGH); 
		ioAlpha.digitalWrite(pinsAlpha::EN3, HIGH); 
		ioAlpha.digitalWrite(pinsAlpha::MUX_EN, LOW); //Turn MUX on 
		ioAlpha.digitalWrite(pinsAlpha::REG_EN, HIGH); //Turn on power to ADC and any 5V ports
		adcConfig(0x71, 0x80); //Configure the ADC to read from channel 3 (MUX in) and use full scale range (6.144V) and 128 sps for high speed 
		float portInputVoltage[3] = {0};
		float portOutputVoltage[3] = {0};
		String portInputString = "\"RAIL_IN\":["; //Sub string for input vals
		String portOutputString = "\"RAIL_OUT\":["; //Sub string for output vals

 		for (int i = 0; i < 3; i++) {
 			ioAlpha.digitalWrite(pinsAlpha::MUX_SEL0, (i & 0x01)); //Write low bit of counter to MUX_SEL0
 			ioAlpha.digitalWrite(pinsAlpha::MUX_SEL1, ((i >> 1) & 0x01)); //Write high bit of counter to MUX_SEL1
			ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, LOW); //Read external ports first
			delay(1); //DEBUG!
			portOutputVoltage[i] = float(adcRead(3, 0))*(0.1875); //Read from port 3 with no gain, convert to mV
			portOutputString = portOutputString + String(portOutputVoltage[i], 4) + ","; //Use max decimal places for min ADC resolution x.1875 
			ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, HIGH); //Read internal ports next
			delay(1); //DEBUG!
			portInputVoltage[i] = float(adcRead(3, 0))*(0.1875); //Read from port 3 with no gain, convert to mV
			portInputString = portInputString + String(portInputVoltage[i], 4) + ","; //Use max decimal places for min ADC resolution x.1875 
			if((portInputVoltage[i] - portOutputVoltage[i])/portInputVoltage[i] > MAX_DISAGREE) throwError(BUS_DISAGREE | i); //Throw port disagree error and note position of port
		}
		
		portInputString = portInputString.substring(0, portInputString.length() - 1) + "],"; //Trim trailing ',' and cap substring
		portOutputString = portOutputString.substring(0, portOutputString.length() - 1) + "]"; //Trim trailing ',' and cap substring

		const float max3v3 = 3300*(1 + MAX_DISAGREE/2.0); //Calc ranges for bus values (working in mV!)
		const float min3v3 = 3300*(1 - MAX_DISAGREE/2.0);
		const float max5v = 5000*(1 + MAX_DISAGREE/2.0);
		const float min5v = 5000*(1 - MAX_DISAGREE/2.0);
		for(int i = 0; i < 3; i++) {
			if(portInputVoltage[i] < max3v3 && portInputVoltage[i] > min3v3) portVoltageSettings[i] = 0; //If within 3v3 range, set port config accordingly 
			else if(portInputVoltage[i] < max5v && portInputVoltage[i] > min5v) portVoltageSettings[i] = 1; //If within the 5v range, set the port config accordingly 
			else {
				if(portInputVoltage[i] < min3v3) portVoltageSettings[i] = 0; //Make assumption about switch position, set config accordingly
				else portVoltageSettings[i] = 1;
				throwError(BUS_OUTOFRANGE | i); //Throw out of range error and note position of port
			}
		}
		ioAlpha.digitalWrite(pinsAlpha::MUX_SEL0, 1); //Connect MUX to 5V rail
		ioAlpha.digitalWrite(pinsAlpha::MUX_SEL1, 1);
		ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, HIGH); 
		float busVoltage_5V = float(adcRead(3, 0))*(0.1875); //Read 5V port with no gain, convert to mV
		if((busVoltage_5V - 5000)/5000 > MAX_DISAGREE) throwError(BUS_OUTOFRANGE | 3); //Throw out of range error and note position of port

		output = output + portInputString + portOutputString + ",\"5V0_RAIL\":" + String(busVoltage_5V, 4) +  "},"; //Concatonate strings and cap
		String level5 = selfDiagnostic(5); //Call the lower level of self diagnostic 
		level5 = level5.substring(1,level5.length() - 1); //Trim off opening and closing brace
		output = output + level5; //Concatonate level 5 on top of level 4
		output = output + "}"; //CLOSE JSON BLOB
		return output;

	}

	else if(diagnosticLevel == 5) {
		String output = "{\"lvl-5\":{"; //OPEN JSON BLOB
		for(int i = 0; i < 3; i++) {
			overflow[i] = ioBeta.getInterrupt(pinsBeta::OVF1 + i); //Read in overflow values
			faults[i] = ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i); //Read in fault values
			if (overflow[i] == true) {
				throwError(COUNTER_OVERFLOW | i); //Throw overflow error with given port appended 
			}
			if (faults[i] == true) {
				throwError(POWER_FAULT | i); //Throw power fault error with given port appended 
			}
		}

		output = output + "\"ALPHA\":" + String(ioAlpha.readBus()) + ","; //Append ALPHA port readout
		output = output + "\"BETA\":" + String(ioBeta.readBus()) + ","; //Append BETA port readout
		output = output + "\"ALPHA_INT\":" + String(ioAlpha.getAllInterrupts(PCAL9535A::IntAge::BOTH)) + ","; //Append ALPHA interrupt readout
		output = output + "\"BETA_INT\":" + String(ioBeta.getAllInterrupts(PCAL9535A::IntAge::BOTH)) + ","; //Append BETA interrupt readout
		output = output + "\"PORT_CFG\":[" + String(portVoltageSettings[0]) + "," + String(portVoltageSettings[1]) + "," + String(portVoltageSettings[2]) + "],"; 
		output = output + "\"I2C\":[";
		for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
			Wire.beginTransmission(adr);
			Wire.write(0x00);
			if(Wire.endTransmission() == 0) {
				output = output + String(adr) + ",";
			}
		}
		if(output.substring(output.length() - 1).equals(",")) {
			output = output.substring(0, output.length() - 1); //Trim trailing ',' is present
		}
		output = output + "]}"; // close array, close pair
		output = output + "}"; //CLOSE JSON BLOB, 
		ioAlpha.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Alpha
		ioBeta.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Beta
		return output;
	}

	return "{}"; //Return null if reach end	
}

int AuxTalon::restart()
{
	bool hasCriticalError = false;
	bool hasError = false;
	if(initDone == false) begin(0, hasCriticalError, hasError); //If for some reason the begin() function has not been run, call this now //FIX!
	setPinDefaults(); //Reset IO expander pins to their default state
	for(int i = 0; i < 3; i++) {
		if (faults[i] == true) { 
			if(ioAlpha.digitalRead(pinsAlpha::FAULT1 + i) == LOW) { //If the FAULT is still asserted 
				ioAlpha.digitalWrite(pinsAlpha::EN1 + i, HIGH); //Turn port power ON
				ioAlpha.digitalWrite(pinsAlpha::EN1 + i, LOW); //Turn port off
				ioAlpha.digitalWrite(pinsAlpha::EN1 + i, HIGH); //Turn port back on finally
				delay(10); //Wait for trip
				if(ioAlpha.digitalRead(pinsAlpha::FAULT1 + i) == LOW) { //If FAULT is re-asserted after power cycle
					throwError(POWER_FAULT_PERSISTENT | i); //Throw persistent power fault error with given port appended 
				}
			}
		}
	}
	return 0; //FIX!
}

String AuxTalon::getData(time_t time)
{
	const time_t startTime = clearTime; //Grab current clear time //FIX! change to report the time used in calculation
	const time_t stopTime = time; //Grab the time the current update is made
	updateCount(time); //Update counter values
	updateAnalog(); //Update analog readings
	
	String output = "{\"OB\":{"; //OPEN JSON BLOB

	String analogData = "\"AIN\":[";
	String analogAvgData = "\"AIN_AVG\":[";
	String countData = "\"COUNTS\":[";
	String rateData = "\"RATE\":[";
	for(int i = 0; i < 3; i++) {
		analogData = analogData + String(analogVals[i], 7) + ",";
		analogAvgData = analogAvgData + String(analogValsAvg[i], 7) + ",";
		countData = countData + String(counts[i]) + ",";
		rateData = rateData + String(rates[i], 7) + ",";
	}
	analogData = analogData.substring(0,analogData.length() - 1) + "],"; //Trim trailing ',' and close array
	analogAvgData = analogAvgData.substring(0,analogAvgData.length() - 1) + "],";
	countData = countData.substring(0,countData.length() - 1) + "],";
	rateData = rateData.substring(0,rateData.length() - 1) + "],";

	output = output + analogData + analogAvgData + countData + rateData; //Concatonate all sub-strings
	output = output + "\"START\":" + String((long) startTime) + ","; //Concatonate start time
	output = output + "\"STOP\":" + String((long) stopTime); //Concatonate stop time
	output = output + "}}"; //CLOSE JSON BLOB
	return output;

}

int AuxTalon::updateCount(time_t time)
{
	time_t tempClearTime = clearTime; //Copy time to calculate rates
	readCounters();
	clearCount(time);
	
	for (int i = 0; i < 3; i++)
	{
		if(timeBaseGood == true) rates[i] = float(counts[i])/float(time - tempClearTime); //Calculate the average rate in Hz, only if there is a good timebase currently
		else rates[i] = 0; //Otherwise, null the rates
	}
	return 0; //DEBUG!
	
	// if(time > readTime && time > clearTime && clearTime != 0 && readTime != 0) { //Make sure new time is logically consistent with previous reading time and clearing time 
	// 	clearTime = readTime; //Swap clear time for read time 
	// 	readTime = time; 
	// 	timeBaseGood = true; //FIX - Check in more aggresive way??
	// }
	// else if(time < readTime || time < clearTime || time == 0) {
	// 	clearTime = 0;
	// 	readTime = 0; 
	// 	timeBaseGood = false; //If any of time times are inconsistent, set the timebase to bad 
	// 	throwError(TIME_BAD);
	// }
	// else if((time - readTime) > maxTimeDelta) {
	// 	clearTime = readTime; //Swap clear time for read time 
	// 	readTime = time; 
	// 	timeBaseGood = false; //Indicate that time base is not secure, but not confident in the failure 
	// 	throwError(TIME_DELTA_EXCEEDED);
	// }
}

int AuxTalon::readCounters()
{
	ioBeta.pinMode(pinsBeta::LOAD, OUTPUT);
	ioBeta.digitalWrite(pinsBeta::LOAD, LOW); //Pulse load line immediately 
	ioBeta.digitalWrite(pinsBeta::LOAD, HIGH);
	ioBeta.digitalWrite(pinsBeta::LOAD, LOW);

	//Configuration should be redudent, kept for robustness //FIX??
	ioBeta.digitalWrite(pinsBeta::COUNT_EN1, HIGH); //Preempt each counter output to disabled state
	ioBeta.digitalWrite(pinsBeta::COUNT_EN2, HIGH); 
	ioBeta.digitalWrite(pinsBeta::COUNT_EN3, HIGH);
	ioBeta.pinMode(pinsBeta::COUNT_EN1, OUTPUT);	//Make sure pinmode is set
	ioBeta.pinMode(pinsBeta::COUNT_EN2, OUTPUT);	
	ioBeta.pinMode(pinsBeta::COUNT_EN3, OUTPUT);
	// ioBeta.pinMode(pinsBeta::OVF1, INPUT); //Make sure overflow lines configured as inputs	
	// ioBeta.pinMode(pinsBeta::OVF2, INPUT);
	// ioBeta.pinMode(pinsBeta::OVF3, INPUT);

	bool resetState = hasReset(); //Store test if device has reset since last init
	for(int i = 0; i < 3; i++) {	
		ioBeta.digitalWrite(pinsBeta::COUNT_EN1 + i, LOW); //Enable given bus
		// overflow[i] = ioBeta.digitalRead(pinsBeta::OVF1 + i); //Read in overflow values
		// overflow[i] = ioBeta.getInterrupt(pinsBeta::OVF1 + i); //Read in overflow values
		// faults[i] = ioAlpha.digitalRead(pinsAlpha::FAULT1 + i); //Read in fault values
		// faults[i] = ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i); //Read in fault values
		if(overflow[i] == false && faults[i] == false && resetState == false) counts[i] = ioGamma.readBus(); //Read parellel bus output
		else counts[i] = 0; //If any critical error present, invalidate data
		ioBeta.digitalWrite(pinsBeta::COUNT_EN1 + i, HIGH); //Disable given bus
		if (resetState == true) {
			throwError(DEVICE_RESET); //Report the unplanned reset
		}
	}
	return 0; //FIX!
}

int AuxTalon::updateCount()
{
	timeBaseGood = false; //If an update is made without passing a time value the timebase becomes invalid 
	return updateCount(0); //Call the general updateCount function but without an accurate timebase 
}

int AuxTalon::clearCount(time_t time)
{
	// if(time > readTime && time > clearTime && clearTime != 0 && readTime != 0) { //Make sure new time is logically consistent with previous reading time and clearing time
	if(time > clearTime && clearTime != 0) { //Make sure new time is logically consistent with previous reading time and clearing time 
		clearTime = time; //Swap clear time for read time  
		timeBaseGood = true; //FIX - Check in more aggresive way??
	}
	// else if(time < readTime || time < clearTime || time == 0) { 
	else if(time < clearTime || time == 0) { 	
		clearTime = 0;
		// readTime = 0; 
		timeBaseGood = false; //If any of time times are inconsistent, set the timebase to bad 
		throwError(TIME_BAD);
	}
	else if((time - clearTime) > maxTimeDelta) {
		clearTime = time; //Copy time value over 
		timeBaseGood = false; //Indicate that time base is not secure, but not confident in the failure 
		throwError(TIME_DELTA_EXCEEDED);
	}

	ioAlpha.pinMode(pinsAlpha::RST, OUTPUT);
	ioAlpha.digitalWrite(pinsAlpha::RST, HIGH); //Pulse RST line
	ioAlpha.digitalWrite(pinsAlpha::RST, LOW);
	ioAlpha.digitalWrite(pinsAlpha::RST, HIGH);
	// ioBeta.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts 
	return 0; //FIX!
}

int AuxTalon::updateAnalog()
{
	//Configure ADC for 128 sps
	for(int i = 0; i < 3; i++) {
			if(ioAlpha.digitalRead(pinsAlpha::FAULT1 + i) == LOW) {

			}
	}
	uint8_t gainVals[3] = {0}; //If not using AGC, default gain to max range
	if(automaticGainControl) {
		for(int port = 0; port < 3; port++) {
			int16_t val = adcRead(port, 1); //Read each port with FSR set to 4.096V
			gainVals[port] = __builtin_clz(val << 1) - 16; //If the measured result is more than half of the FSR at a given step, choose the next highest gain range. Measuring the number of leading zeros tells us the cap of the size of the reading in a divisable by 2 manor. Need to subtract 16 since it works in a 32 bit world
			if(gainVals[port] > sizeof(adcGainConfigs)/sizeof(adcGainConfigs[0])) { //FIX! If result is greater than max index of array, force to max 0, can occour if I2C error
				gainVals[port] = 0;
				//FIX!
			}
		}
	}

	for(int port = 0; port < 3; port++) {
		analogVals[port] = float(adcRead(port, gainVals[port]))*adcGainConv[gainVals[port]]; //Report converted value
		// analogVals[port] = gainVals[port]; //Report converted value //DEBUG!
		long accumulatorVal = 0; //Used to sum all measures
		for(int i = 0; i < samplesToAverage; i++) { //Sum all N samples together
			accumulatorVal = accumulatorVal + adcRead(port, gainVals[port]);
		}
		analogValsAvg[port] = ((float(accumulatorVal)*adcGainConv[gainVals[port]])/float(samplesToAverage)); //divide by total number of samples taken, multiply by converting value, perform math in float form to prevent rounding error on int division
	}
	return 0; //FIX!
}

int16_t AuxTalon::adcRead(uint8_t port, uint8_t gain)
{
	int error = adcConfig(adcBaseConfigHigh | adcGainConfigs[gain] | adcPortConfigs[port], adcBaseConfigLow); //Set config to base with specified gain and connected to given port
	adcConfig(adcBaseConfigHigh | adcGainConfigs[gain] | adcPortConfigs[port] | adcStartConversion, adcBaseConfigLow); //Tell ADC to begin conversion 
	if(error != 0){
		throwError(ADC_I2C_ERROR | error); //Alert to error if I2C comunication problem 
		return 0; //Exit with error condition 
	}
	bool newRead = (readADCReg(0x01) >> 15) & 0x01; //Grab OS bit from config reg
	unsigned long localTime = millis();
	while(!newRead && (millis() - localTime) < adcReadTimeout) { //if there is not already a new read, continue to read from config reg until new read is reported or timeout occours 
		newRead = (readADCReg() >> 15) & 0x01 ;
	}
	if(!newRead) {
		throwError(ADC_TIMEOUT_ERROR); //If new reading is not done, throw timeout error
		return 0; //Exit with error condition 
	}
	else {
		return readADCReg(0x00); //Return read of conversion register 
	}
	// Wire.beginTransmission(WireTransmission(ADR_ADS1115).timeout(adcReadTimeout));
	// Wire.write(0x01); //Point to configuration register 
	// error = Wire.endTransmission();
	// if(error != 0) {
	// 	throwError(ADC_I2C_ERROR | error); //Alert to error if I2C comunication problem 
	// 	return -9999.0; //Exit with error condition 
	// }

	// bool newRead = false; //Keep track of single shot bit
	// uint8_t configHigh = 0; //Store local copies of config read
	// uint8_t configLow = 0; 
	// uint8_t numBytes = 0; //Number of bytes read from the ADC
	// unsigned long localTime = millis();
	// while(error == 0 && (millis() - localTime) < adcReadTimeout && !newRead) { //Keep polling until there is a new reading, or an error occours, or a timeout occours 
	// 	numBytes = Wire.requestFrom(WireTransmission(ADR_ADS1115).quantity(2).timeout(adcReadTimeout));
	// 	// while(Wire.available() < 2 && (millis() - localTime) < adcReadTimeout)
	// 	if(numBytes == 2) {
	// 		configHigh = Wire.read();
	// 		configLow = Wire.read();
	// 		newRead = configHigh >> 7; //Grab OS bit from config reg
	// 	}
	// 	else error = 0x07; //Set timeout error 
	// }
	
}

int AuxTalon::adcConfig(uint8_t configHigh, uint8_t configLow)
{
	Wire.beginTransmission(ADR_ADS1115);
	Wire.write(0x01); //Set pointer to config register
	Wire.write(configHigh); //Write high byte
	Wire.write(configLow); //Write low byte
	int error = Wire.endTransmission();
	return error; 
}

int16_t AuxTalon::readADCReg(uint8_t reg)
{
	Wire.beginTransmission(WireTransmission(ADR_ADS1115).timeout(adcReadTimeout));
	Wire.write(reg); //Point to desired register 
	int error = Wire.endTransmission();
	if(error != 0) {
		throwError(ADC_I2C_ERROR | error); //Alert to error if I2C comunication problem 
		return 0; //Exit with error condition 
	}

	uint8_t valHigh = 0; //Store local copies of reg 
	uint8_t valLow = 0; 
	uint8_t numBytes = 0; //Number of bytes read from the ADC
	numBytes = Wire.requestFrom(WireTransmission(ADR_ADS1115).quantity(2).timeout(adcReadTimeout));
	if(numBytes == 2) {
		valHigh = Wire.read();
		valLow = Wire.read();
		return (valHigh << 8) | valLow; //Concatonate and return correct output
	}
	else {
		error = 0x07; //Set timeout error 
		throwError(ADC_I2C_ERROR | error); //If I2C timeout occoured, throw I2C error as well 
		return 0;
	}
}

int16_t AuxTalon::readADCReg()
{
	uint8_t valHigh = 0; //Store local copies of reg 
	uint8_t valLow = 0; 
	uint8_t numBytes = 0; //Number of bytes read from the ADC
	numBytes = Wire.requestFrom(WireTransmission(ADR_ADS1115).quantity(2).timeout(adcReadTimeout));
	if(numBytes == 2) {
		valHigh = Wire.read();
		valLow = Wire.read();
		return (valHigh << 8) | valLow; //Concatonate and return correct output
	}
	else {
		int error = 0x07; //Set timeout error 
		throwError(ADC_I2C_ERROR | error); //If I2C timeout occoured, throw I2C error as well 
		return 0;
	}
}

void AuxTalon::setPinDefaults()
{
	/////////////////////////////// Initialize IO ALPHA Pins /////////////////////////////////
	ioAlpha.pinMode(pinsAlpha::MUX_EN, OUTPUT); //THIS CONFIGURATION IS ONLY MADE HERE, USED AS A HARDWARE SETUP TEST ELSEWHERE!
	ioAlpha.digitalWrite(pinsAlpha::MUX_EN, HIGH); //Turn MUX off by default

	ioAlpha.pinMode(pinsAlpha::MUX_SEL0, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::MUX_SEL1, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::MUX_SEL2, OUTPUT);

	ioAlpha.digitalWrite(pinsAlpha::MUX_SEL0, LOW);
	ioAlpha.digitalWrite(pinsAlpha::MUX_SEL1, LOW);
	ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, LOW);

	ioAlpha.digitalWrite(pinsAlpha::ACTRL1, LOW); //Preempt before output enable
	ioAlpha.digitalWrite(pinsAlpha::ACTRL2, LOW); //Preempt before output enable
	ioAlpha.digitalWrite(pinsAlpha::ACTRL3, LOW); //Preempt before output enable

	ioAlpha.pinMode(pinsAlpha::ACTRL1, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::ACTRL2, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::ACTRL3, OUTPUT);

	ioAlpha.pinMode(pinsAlpha::FAULT1, INPUT_PULLUP); 
	ioAlpha.pinMode(pinsAlpha::FAULT2, INPUT_PULLUP);
	ioAlpha.pinMode(pinsAlpha::FAULT3, INPUT_PULLUP);

	ioAlpha.pinMode(pinsAlpha::EN1, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::EN2, OUTPUT);
	ioAlpha.pinMode(pinsAlpha::EN3, OUTPUT);

	ioAlpha.digitalWrite(pinsAlpha::EN1, HIGH); //Default to ON
	ioAlpha.digitalWrite(pinsAlpha::EN2, HIGH); //Default to ON
	ioAlpha.digitalWrite(pinsAlpha::EN3, HIGH); //Default to ON

	ioAlpha.pinMode(pinsAlpha::ADC_INT, INPUT_PULLUP); 

	ioAlpha.pinMode(pinsAlpha::REG_EN, OUTPUT);
	ioAlpha.digitalWrite(pinsAlpha::REG_EN, HIGH); //Turn on 5V converter 

	ioAlpha.pinMode(pinsAlpha::RST, OUTPUT);
	ioAlpha.digitalWrite(pinsAlpha::RST, HIGH); //Default to negate reset

	/////////////////////////////// Initialize IO BETA Pins /////////////////////////////////
	ioBeta.pinMode(pinsBeta::OUT1, INPUT);
	ioBeta.pinMode(pinsBeta::OUT2, INPUT);
	ioBeta.pinMode(pinsBeta::OUT3, INPUT);

	ioBeta.pinMode(pinsBeta::OVF1, INPUT);
	ioBeta.pinMode(pinsBeta::OVF2, INPUT);
	ioBeta.pinMode(pinsBeta::OVF3, INPUT);

	ioBeta.pinMode(pinsBeta::D1_SENSE, INPUT);
	ioBeta.pinMode(pinsBeta::D2_SENSE, INPUT);
	ioBeta.pinMode(pinsBeta::D3_SENSE, INPUT);

	ioBeta.pinMode(pinsBeta::OD1, INPUT);
	ioBeta.pinMode(pinsBeta::OD2, INPUT);
	ioBeta.pinMode(pinsBeta::OD3, INPUT);

	ioBeta.digitalWrite(pinsBeta::LOAD, LOW); //Preempt before output enable
	ioBeta.pinMode(pinsBeta::LOAD, OUTPUT);

	ioBeta.digitalWrite(pinsBeta::COUNT_EN1, LOW); //Preempt before output enable
	ioBeta.digitalWrite(pinsBeta::COUNT_EN2, LOW); //Preempt before output enable
	ioBeta.digitalWrite(pinsBeta::COUNT_EN3, LOW); //Preempt before output enable

	ioBeta.pinMode(pinsBeta::COUNT_EN1, OUTPUT);
	ioBeta.pinMode(pinsBeta::COUNT_EN2, OUTPUT);
	ioBeta.pinMode(pinsBeta::COUNT_EN3, OUTPUT);

	ioBeta.setLatch(pinsBeta::OVF1, true); //Turn on latching for all OVF interrupts
	ioBeta.setLatch(pinsBeta::OVF2, true);
	ioBeta.setLatch(pinsBeta::OVF3, true);

	ioBeta.setInterrupt(pinsBeta::OVF1, true); //Turn on interrupts for all OVF pins
	ioBeta.setInterrupt(pinsBeta::OVF2, true);
	ioBeta.setInterrupt(pinsBeta::OVF3, true);

	ioAlpha.setLatch(pinsAlpha::FAULT1, true); //Turn on latching for fault inputs
	ioAlpha.setLatch(pinsAlpha::FAULT2, true);
	ioAlpha.setLatch(pinsAlpha::FAULT3, true);

	ioAlpha.setInterrupt(pinsAlpha::FAULT1, true); //Turn on interrupts for all FAULT pins
	ioAlpha.setInterrupt(pinsAlpha::FAULT2, true);
	ioAlpha.setInterrupt(pinsAlpha::FAULT3, true);
	
}

bool AuxTalon::hasReset()
{
	int error = 0; //Used to store the error from I2C read
	uint16_t outputState = ioAlpha.readWord(0x06, error); //Read from configuration register 
	if(((outputState >> pinsAlpha::MUX_EN) & 0x01) == 0x00) return false; //If output is still 
	else return true; //If the MUX_EN pin is no longer configured as an output, assume the Talon has reset
}
