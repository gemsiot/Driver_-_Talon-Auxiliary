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

AuxTalon::AuxTalon(uint8_t talonPort_, uint8_t hardwareVersion) : ioAlpha(0x20), ioBeta(0x23), ioGamma(0x24)
{
	// talonPort = talonPort_ - 1; //Copy to local //FIX!
	if(talonPort_ > 0) talonPort = talonPort_ - 1;
	else talonPort = 255; //Reset to null default if not in range
	version = hardwareVersion; //Copy to local
	talonInterface = BusType::NONE; 
	keepPowered = true; //DEBUG! Force this device on
}

String AuxTalon::begin(time_t time, bool &criticalFault, bool &fault) 
{
	//Only use isEnabled() if using particle
	// #if defined(ARDUINO) && ARDUINO >= 100 
	// 	Wire.begin();
	// #elif defined(PARTICLE)
	if(!Wire.isEnabled()) Wire.begin(); //Only initialize I2C if not done already //INCLUDE FOR USE WITH PARTICLE 
	// #endif
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
			throwError(IO_INIT_FAIL | (ioError[i] << 8) | talonPortErrorCode | i + 1); //Throw error on first init error, not again 
			criticalFault = true; //If any IO expander fails, this is a critical error  
			break;
		}
	}

	Wire.beginTransmission(ADR_ADS1115);
	Wire.write(0x00);
	int adcError = Wire.endTransmission(); 
	if(adcError != 0) {
		throwError(AUX_ADC_INIT_FAIL | (adcError << 8) | talonPortErrorCode); //Throw ADC initialization error
		fault = true; //Set non-critical fault flag
	}
	// ads.begin();

	setPinDefaults();
	
	///////////////////// RUN DIAGNOSTICS /////////////
	// String diagnosticResults = selfDiagnostic(2); //Run level two diagnostic

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
	// return diagnosticResults; //Return diagnostic string
	return "";
}

// int AuxTalon::restart()
// {
// 	int startingErrors = numErrors; //Grab the number of errors which have been logged when we start the begin call, used to keep track of new errors
// 	//Initalize io expanders 
// 	int ioError[3] = {0};
// 	ioError[0] = ioAlpha.begin();
// 	ioError[1] = ioBeta.begin();
// 	ioError[2] = ioGamma.begin();	

// 	for(int i = 0; i < 3; i++) {
// 		if(ioError[i] != 0) { 
// 			throwError(IO_INIT_ERROR | ioError[i]); //Throw error on first init error, not again 
// 			// criticalFault = true; //If any IO expander fails, this is a critical error  
// 			break;
// 		}
// 	}

// 	Wire.beginTransmission(ADR_ADS1115);
// 	Wire.write(0x00);
// 	if(Wire.endTransmission() != 0) {
// 		throwError(ADC_INIT_ERROR); //Throw ADC initialization error
// 		// fault = true; //Set non-critical fault flag
// 	}
// 	// ads.begin();

// 	setPinDefaults();
// }

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
	
	String output = "\"Talon-Aux\":{"; // OPEN JSON BLOB
	output = output + "\"CODES\":["; //Open codes pair

	for(int i = 0; i < min(MAX_NUM_ERRORS, numErrors); i++) { //Interate over used element of array without exceeding bounds
		output = output + "\"0x" + String(errors[i], HEX) + "\","; //Add each error code
		errors[i] = 0; //Clear errors as they are read
	}
	if(output.substring(output.length() - 1).equals(",")) {
		output = output.substring(0, output.length() - 1); //Trim trailing ','
	}
	output = output + "],"; //close codes pair
	output =  output + "\"OW\":"; //Open state pair
	if(numErrors > MAX_NUM_ERRORS) output = output + "1,"; //If overwritten, indicate the overwrite is true
	else output = output + "0,"; //Otherwise set it as clear
	output = output + "\"NUM\":" + String(numErrors) + ","; //Append number of errors
	output = output + "\"Pos\":[" + getTalonPortString() + "]"; //Concatonate position 
	output = output + "}"; //CLOSE JSON BLOB
	numErrors = 0; //Clear error count
	return output;

	// return -1; //Return fault if unknown cause 
}

// int AuxTalon::throwError(uint32_t error)
// {
// 	errors[(numErrors++) % MAX_NUM_ERRORS] = error; //Write error to the specified location in the error array
// 	if(numErrors > MAX_NUM_ERRORS) errorOverwrite = true; //Set flag if looping over previous errors 
// 	return numErrors;
// }

String AuxTalon::selfDiagnostic(uint8_t diagnosticLevel, time_t time)
{
	unsigned long diagnosticStart = millis(); 
	bool talonPresent = true;
	if(getTalonPort() == 0) {
		talonPresent = false; //Clear flag //DEBUG!
		throwError(TALON_MISSING); //If Talon not found, report failure
	}
	// if(getTalonPort() == 0) return "{\"Talon-Aux\":null}"; //Return null result if there is no port detected for Talon
	String output = "\"Talon-Aux\":{";
	String sysOutput = ""; //Generate a string for the system level info 
	String portOutput[numPorts] = {""}; //Generate a seperate string for each port
	for(int i = 0; i < numPorts; i++) portOutput[i] = "\"PORT_" + String(i + 1) + "\":{"; //Generate header for each port subgroup
	if(diagnosticLevel == 0) {
		//TBD
		// output = output + "\"lvl-0\":{},\"Pos\":[" + getTalonPortString() + "]}},";
		// return output + "\"lvl-0\":{},\"Pos\":[" + String(port) + "]}}";
	}

	if(diagnosticLevel <= 1) {
		//TBD
		// output = output + "\"lvl-1\":{},\"Pos\":[" + getTalonPortString() + "]}},";
		const int numPulses = 65536; //Number of pulses to use for testing
		for(int i = 0; i < 3; i++) {
			ioBeta.digitalWrite(pinsBeta::OD1 + i, LOW); //Preempt the output as low
			ioBeta.pinMode(pinsBeta::OD1 + i, OUTPUT); //Set line to output	
			ioBeta.digitalWrite(pinsBeta::D1_SENSE + i, LOW); //Preempt low
			ioBeta.pinMode(pinsBeta::D1_SENSE + i, OUTPUT); //Set to output to drive push-pull
			ioBeta.pinMode(pinsBeta::OUT1 + i, INPUT); //Set as input so we can measure the output of the input buffer
		}
		clearCount(time); //Clear counters to start with 0 value
		// bool inputErrorA = false; //Used to keep track if there is an error in the input driver circuit
		// bool inputErrorB = false;
		for(int port = 0; port < 3; port++) {
			Serial.print("Testing AUX Counter "); //DEBUG!
			Serial.println(port + 1); 
			ioBeta.safeMode(PCAL9535A::SAFEOFF); //Turn off for fastest write
			for(int p = 1; p < numPulses; p++) { //Pulse input 5 times
				// if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != HIGH) inputErrorA = true; //If the OUTx line is not sitting high, there is an error in the input circuit
				ioBeta.digitalWrite(pinsBeta::D1_SENSE + port, HIGH);
				// if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != LOW) inputErrorB = true; //If after toggling the Dx input high, the output does not go low there is an error in the input circuit 
				delayMicroseconds(10);
				ioBeta.digitalWrite(pinsBeta::D1_SENSE + port, LOW);
				delayMicroseconds(10);
				readCounters(); //Read in values after testing
				if(counts[port] != p) throwError(COUNTER_INCREMENT_FAIL | 0x0200 | portErrorCode | (port + 1)); //If increment does not match, throw error
				Serial.print(counts[port]); //DEBUG!
				Serial.print(","); 
				Serial.println(p);
			}
			
			// clearCount(time);
			//Send previously obtained error codes
			// if(inputErrorA) throwError(INPUT_BUFF_FAIL | 0x0100 | portErrorCode | (port + 1)); //OR with fail to force low error code
			// if(inputErrorB) throwError(INPUT_BUFF_FAIL | 0x0200 | portErrorCode | (port + 1)); //OR with fail to force high error code
			
			// else if(counts[port] != numPulses) { //If OUTx line responded as expected, but we STILL did not end up with the correct count, then this is a counter problem 
			// 	throwError(COUNTER_INCREMENT_FAIL | portErrorCode | (port + 1)); 
			// 	// for(int i = 0; i < 3; i++) { //DEBUG!
			// 	// 	Serial.println(counts[i]); 
			// 	// }
			// }	
			// inputErrorA = false; //Reset input error
			// inputErrorB = false;
		}
		clearCount(time); //Clear counters again
	}

	if(diagnosticLevel <= 2) {
		//TBD
		// output = output + "\"lvl-2\":{\"Dummy\":\"THisisadummystringforthepusrposesoftestingthewraparoundfunctionalityoftheparserfordealingwithveryveryverylongstringsofstuffTHisisadummystringforthepusrposesoftestingthewraparoundfunctionalityoftheparserfordealingwithveryveryverylongstringsofstuff\"},";
		// String level3 = selfDiagnostic(3, time); //Call the lower level of self diagnostic 
		// level3 = level3.substring(1,level3.length() - 1); //Trim off opening and closing brace
		// output = output + level3; //Concatonate level 4 on top of level 3
		// output = output + "},"; //CLOSE JSON BLOB
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;
		// return "{\"lvl-2\":{}," + selfDiagnostic(3, time).substring(0, ) }";
		pinMode(KestrelPins::PortAPins[talonPort], INPUT); //DEBUG!
	}

	if(diagnosticLevel <= 3) {
		//TBD
		// Serial.println(millis()); //DEBUG!
		// output = output + "\"lvl-3\":{"; //OPEN JSON BLOB
		///////// TEST INPUT DRIVES //////////////
		const int numPulses = 5; //Number of pulses to use for testing
		for(int i = 0; i < 3; i++) {
			ioBeta.digitalWrite(pinsBeta::OD1 + i, LOW); //Preempt the output as low
			ioBeta.pinMode(pinsBeta::OD1 + i, OUTPUT); //Set line to output	
			ioBeta.digitalWrite(pinsBeta::D1_SENSE + i, LOW); //Preempt low
			ioBeta.pinMode(pinsBeta::D1_SENSE + i, OUTPUT); //Set to output to drive push-pull
			ioBeta.pinMode(pinsBeta::OUT1 + i, INPUT); //Set as input so we can measure the output of the input buffer
		}
		clearCount(time); //Clear counters to start with 0 value
		bool inputErrorA = false; //Used to keep track if there is an error in the input driver circuit
		bool inputErrorB = false;
		for(int port = 0; port < 3; port++) {
			for(int p = 0; p < numPulses; p++) { //Pulse input 5 times
				if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != HIGH) inputErrorA = true; //If the OUTx line is not sitting high, there is an error in the input circuit
				ioBeta.digitalWrite(pinsBeta::D1_SENSE + port, HIGH);
				if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != LOW) inputErrorB = true; //If after toggling the Dx input high, the output does not go low there is an error in the input circuit 
				delay(1);
				ioBeta.digitalWrite(pinsBeta::D1_SENSE + port, LOW);
				delay(1);
			}
			readCounters(); //Read in values after testing
			// clearCount(time);
			//Send previously obtained error codes
			if(inputErrorA) throwError(INPUT_BUFF_FAIL | 0x0100 | portErrorCode | (port + 1)); //OR with fail to force low error code
			else if(inputErrorB) throwError(INPUT_BUFF_FAIL | 0x0200 | portErrorCode | (port + 1)); //OR with fail to force high error code
			else if(counts[port] == 0) throwError(COUNTER_INCREMENT_FAIL | 0x0100 | portErrorCode | (port + 1)); //Indicate counter does not increment at all
			else if(counts[port] != numPulses) { //If OUTx line responded as expected, but we STILL did not end up with the correct count, then this is a counter problem 
				throwError(COUNTER_INCREMENT_FAIL | 0x0300 | portErrorCode | (port + 1)); 
				// for(int i = 0; i < 3; i++) { //DEBUG!
				// 	Serial.println(counts[i]); 
				// }
			}	
			inputErrorA = false; //Reset input error
			inputErrorB = false;
		}
		clearCount(time); //Clear counters again
		readCounters(); //Read in values after clearing 
		if(counts[0] != 0 || counts[1] != 0 || counts[2] != 0) throwError(COUNTER_CLEAR_FAIL | portErrorCode); //If counter does not clear correctly, throw error 
		// inputError = false; //Clear error flag for next test 
		
		for(int i = 0; i < 3; i++) {
			ioBeta.digitalWrite(pinsBeta::D1_SENSE + i, LOW); //Drive all Dx_SENSE lines low to prevent erronious output ticks
			ioBeta.digitalWrite(pinsBeta::OD1 + i, LOW); //Drive all ODx lines low to start
		}
	
		for(int port = 0; port < 3; port++) {
			for(int p = 0; p < numPulses; p++) { //Pulse input 5 times
				if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != HIGH) inputErrorA = true; //If the OUTx line is not sitting low, there is an error in the input circuit
				// ioBeta.digitalWrite(pinsBeta::OD1 + port, HIGH);
				ioBeta.pinMode(pinsBeta::OD1 + port, INPUT); //Switch ODx to input to release to pullup (do this instead of push-pull to prevent output shorting)
				delay(1);
				if(ioBeta.digitalRead(pinsBeta::OUT1 + port) != LOW) inputErrorB = true; //If after toggling the ODx input high, the output does not go low there is an error in the input circuit //FIX! switch to interrupt measurment for this part
				delay(1);
				// ioBeta.digitalWrite(pinsBeta::OD1 + port, LOW);
				ioBeta.pinMode(pinsBeta::OD1 + port, OUTPUT); //Turn on ODx output to drive low
				delay(1);
			}
			// readCounters(); //Read in values after testing
			if(inputErrorA) throwError(INPUT_BUFF_FAIL | 0x0100 | portErrorCode | (port + 1)); //OR with fail to force low error code
			if(inputErrorB) throwError(INPUT_BUFF_FAIL | 0x0300 | portErrorCode | (port + 1)); //OR with fail on release error code
		}

		///////////// IDENTIFY Dx INPUTS //////////
		bool digitalInputOccupied[3] = {false}; //Used to store results of digital input testing
		int digitalInputCurrentState[3] = {0}; //Used to store current state of digital input (0 = LOW, 1 = HIGH, -1 = N/A)

		bool pullupVal = 0;
		bool pulldownVal = 0;
		for(int port = 0; port < numPorts; port++) {
			ioBeta.pinMode(pinsBeta::D1_SENSE + port, INPUT_PULLUP); //Set given digital input pin as pullup
			delay(1); //Wait for line to charge if floating
			pullupVal = ioBeta.digitalRead(pinsBeta::D1_SENSE + port); //Read state when pullup is applied
			ioBeta.pinMode(pinsBeta::D1_SENSE + port, INPUT_PULLDOWN); //Set given digital input pin as pulldown
			delay(1);
			pulldownVal = ioBeta.digitalRead(pinsBeta::D1_SENSE + port); //Read state when pullup is disconnected 
			if(pullupVal == HIGH && pulldownVal == LOW) digitalInputOccupied[port] = false;
			else digitalInputOccupied[port] = true;

			if(digitalInputOccupied[port] == true) {
				ioBeta.pinMode(pinsBeta::D1_SENSE, INPUT); //Return to high impedance input
				digitalInputCurrentState[port] = ioBeta.digitalRead(pinsBeta::D1_SENSE + port); //Read in current value
			}
			else digitalInputCurrentState[port] = -1; //Set to N/A if the port is unoccupied  
		}

		// String occupied = "\"Dx_USE\":[";
		// String currentState = "\"Dx_STATE\":[";
		// for(int i = 0; i < 3; i++) {
		// 	occupied = occupied + String(digitalInputOccupied[i]) + ",";
		// 	currentState = currentState + String(digitalInputCurrentState[i]) + ",";
		// }
		// occupied = occupied.substring(0, occupied.length() - 1) + "],"; //Trim off trailing ',' and close
		// currentState = currentState.substring(0, currentState.length() - 1) + "],"; //Trim off trailing ',' and close
		// output = output + occupied + currentState; //Concatonate together


		for(int i = 0; i < 3; i++) { //Return pins to defaults
			ioBeta.pinMode(pinsBeta::OD1 + i, INPUT); //Return to input	
			ioBeta.pinMode(pinsBeta::D1_SENSE + i, INPUT); //Return to input
			ioBeta.pinMode(pinsBeta::OUT1 + i, INPUT); //Keep as input
		}
		clearCount(time); //Clear counters again //Total time between start and this clear event is < 300ms, ok to use same timestamp
		// Serial.println(millis()); //DEBUG!

		////////////// TEST ANALOG OUTPUTS /////////////////
		float senseOpen[3] = {0};
		float senseDischarged[3] = {0};
		float senseLoaded[3] = {0};
		unsigned long dischargePeriod = 250; //Time to wait while discharging in ms
		uint8_t offsetSamples = 16; //Number of samples to take to average value if measuring the offset value 
		float offsetThreshold = 1.0; //Threshold for reading offset value

		for(int port = 0; port < 3; port++) {
			senseOpen[port] = float(adcRead(port, 0))*(adcGainConv[0]); //Read baseline port value at full range
			ioAlpha.pinMode(pinsAlpha::ACTRL1 + port, OUTPUT); //Set MOSFET drive to output
			ioAlpha.digitalWrite(pinsAlpha::ACTRL1 + port, HIGH); //Turn on MOSFET to discharge output
			delay(dischargePeriod); //Wait for RC circuit to discharge
			senseLoaded[port] = float(adcRead(port, 0))*(adcGainConv[0]); //Read fully discharged value 
			// Serial.print("Loaded Value - Raw: "); //DEBUG!
			// Serial.print(adcRead(port, 0)); //Dummy read to clear //DEBUG!
			// Serial.print("\t"); //DEBUG!
			// Serial.println(adcRead(port, 5)); //Dummy read to clear //DEBUG!
			// delay(150); //DEBUG!
			// senseLoaded[port] = float(adcRead(port, 5))*(adcGainConv[5]); //Read fully discharged value, high gain //DEBUG!
			if(senseLoaded[port] < offsetThreshold) {
				float offsetMeasure = 0; //Used to temporarily store offset measure
				adcRead(port, 5); //Dummy read to ensure register is clear 
				for(int i = 0; i < offsetSamples; i++) {
					offsetMeasure = offsetMeasure + float(adcRead(port, 5))*(adcGainConv[5]); //Read at high gain
				}
				senseLoaded[port] = offsetMeasure/float(offsetSamples); //Set loaded value with averaged value
			}
			ioAlpha.digitalWrite(pinsAlpha::ACTRL1 + port, LOW); //Turn MOSFET off to release line
			senseDischarged[port] = float(adcRead(port, 0))*(adcGainConv[0]); //Read in discharged value
			// Serial.print("AuxTalonSenseVals: Port ");
			// Serial.print(port);
			// Serial.print(senseOpen[port]);
			// Serial.print("\t");
			// Serial.print(senseLoaded[port]);
			// Serial.print("\t");
			// Serial.println(senseDischarged[port]);

		}
		for(int i = 0; i < numPorts; i++) {
			if(talonPresent) portOutput[i] = portOutput[i] + "\"In_Occ\":" + String(digitalInputOccupied[i]) + ",\"In_State\":" + String(digitalInputCurrentState[i]) + ",\"Ain_O\":" + String(senseOpen[i]) + ",\"Ain_D\":" + String(senseDischarged[i]) + ",\"Ain_L\":" + String(senseLoaded[i]) + ",";
			else portOutput[i] = portOutput[i] + "\"In_Occ\":null,\"In_State\":null,\"Ain_O\":null,\"Ain_D\":null,\"Ain_L\":null,";
		}
		
		// output = output + "\"AIN_SENSE\":{"; //Open array

		// String open = "\"OPEN\":[";
		// String discharged = "\"DIS\":[";
		// String loaded = "\"LOAD\":[";

		// for(int i = 0; i < 3; i++) {
		// 	open = open + String(senseOpen[i], 4) + ",";
		// 	discharged = discharged + String(senseDischarged[i], 4) + ",";
		// 	loaded = loaded + String(senseLoaded[i], 4) + ",";
		// }
		// open = open.substring(0, open.length() - 1) + "],"; //Trim off trailing ',' and close
		// discharged = discharged.substring(0, discharged.length() - 1) + "],"; //Trim off trailing ',' and close
		// loaded = loaded.substring(0, loaded.length() - 1) + "]"; //Trim off trailing ',' and close

		// output = output + open + discharged + loaded + "}},"; //Close AIN_SENSE
		// String level4 = selfDiagnostic(4); //Call the lower level of self diagnostic 
		// level4 = level4.substring(1,level4.length() - 1); //Trim off opening and closing brace
		// output = output + level4; //Concatonate level 4 on top of level 3
		// output = output + "},"; //CLOSE JSON BLOB
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;

 	}

	if(diagnosticLevel <= 4) {
		// String output = selfDiagnostic(5); //Call the lower level of self diagnostic 
		// output = output.substring(0,output.length() - 1); //Trim off closing brace
		// output = output + "\"lvl-4\":{"; //OPEN JSON BLOB

		ioAlpha.digitalWrite(pinsAlpha::EN1, HIGH); //Make sure all ports are enabled before testing 
		ioAlpha.digitalWrite(pinsAlpha::EN2, HIGH); 
		ioAlpha.digitalWrite(pinsAlpha::EN3, HIGH); 
		ioAlpha.digitalWrite(pinsAlpha::MUX_EN, LOW); //Turn MUX on 
		ioAlpha.digitalWrite(pinsAlpha::REG_EN, HIGH); //Turn on power to ADC and any 5V ports
		adcConfig(0x71, 0x80); //Configure the ADC to read from channel 3 (MUX in) and use full scale range (6.144V) and 128 sps for high speed 
		float portInputVoltage[3] = {0};
		float portOutputVoltage[3] = {0};
		// String portInputString = "\"RAIL_IN\":["; //Sub string for input vals
		// String portOutputString = "\"RAIL_OUT\":["; //Sub string for output vals

 		for (int i = 0; i < 3; i++) {
 			ioAlpha.digitalWrite(pinsAlpha::MUX_SEL0, (i & 0x01)); //Write low bit of counter to MUX_SEL0
 			ioAlpha.digitalWrite(pinsAlpha::MUX_SEL1, ((i >> 1) & 0x01)); //Write high bit of counter to MUX_SEL1
			ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, LOW); //Read external ports first
			delay(1); //DEBUG!
			portOutputVoltage[i] = float(adcRead(3, 0))*(adcGainConv[0]); //Read from port 3 with no gain, convert to mV
			// portOutputString = portOutputString + String(portOutputVoltage[i], 4) + ","; //Use max decimal places for min ADC resolution x.1875 
			ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, HIGH); //Read internal ports next
			delay(1); //DEBUG!
			portInputVoltage[i] = float(adcRead(3, 0))*(adcGainConv[0]); //Read from port 3 with no gain, convert to mV
			// portInputString = portInputString + String(portInputVoltage[i], 4) + ","; //Use max decimal places for min ADC resolution x.1875 
			if((portInputVoltage[i] - portOutputVoltage[i])/portInputVoltage[i] > MAX_DISAGREE) throwError(BUS_DISAGREE | talonPortErrorCode | i + 1); //Throw port disagree error and note position of port
		}
		
		// portInputString = portInputString.substring(0, portInputString.length() - 1) + "],"; //Trim trailing ',' and cap substring
		// portOutputString = portOutputString.substring(0, portOutputString.length() - 1) + "]"; //Trim trailing ',' and cap substring

		const float max3v3 = 3300*(1 + MAX_DISAGREE/2.0); //Calc ranges for bus values (working in mV!)
		const float min3v3 = 3300*(1 - MAX_DISAGREE/2.0);
		const float max5v = 5000*(1 + MAX_DISAGREE/2.0);
		const float min5v = 5000*(1 - MAX_DISAGREE/2.0);
		for(int i = 0; i < 3; i++) {
			if(portInputVoltage[i] < max3v3 && portInputVoltage[i] > min3v3) portVoltageSettings[i] = 0; //If within 3v3 range, set port config accordingly 
			else if(portInputVoltage[i] < max5v && portInputVoltage[i] > min5v) portVoltageSettings[i] = 1; //If within the 5v range, set the port config accordingly 
			else {
				if(portInputVoltage[i] < min3v3) {
					portVoltageSettings[i] = 0; //Make assumption about switch position, set config accordingly
					throwError(BUS_OUTOFRANGE | 0x100 | talonPortErrorCode | i + 1); //Throw undervolt error
				}
				else {
					portVoltageSettings[i] = 1;
					throwError(BUS_OUTOFRANGE | talonPortErrorCode | i + 1); //Throw general out of range error and note position of port
				}
			}
		}
		ioAlpha.digitalWrite(pinsAlpha::MUX_SEL0, 1); //Connect MUX to 5V rail
		ioAlpha.digitalWrite(pinsAlpha::MUX_SEL1, 1);
		ioAlpha.digitalWrite(pinsAlpha::MUX_SEL2, HIGH); 
		float busVoltage_5V = float(adcRead(3, 0))*(adcGainConv[0]); //Read 5V port with no gain, convert to mV
		if(busVoltage_5V < min5v) throwError(BUS_OUTOFRANGE | 0x100 | talonPortErrorCode | 4); //Throw undervolt error
		if(abs(busVoltage_5V - 5000)/5000 > MAX_DISAGREE) throwError(BUS_OUTOFRANGE | talonPortErrorCode | 4); //Throw general out of range error and note position of port

		// output = output + portInputString + portOutputString + ",\"5V0_RAIL\":" + String(busVoltage_5V, 4) +  "},"; //Concatonate strings and cap
		// String level5 = selfDiagnostic(5); //Call the lower level of self diagnostic 
		// level5 = level5.substring(1,level5.length() - 1); //Trim off opening and closing brace
		// output = output + level5; //Concatonate level 5 on top of level 4
		// output = output + "},"; //CLOSE JSON BLOB
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;
		for(int i = 0; i < numPorts; i++) {
			if(talonPresent) portOutput[i] = portOutput[i] + "\"Vi\":" + String(portInputVoltage[i]) + ",\"Vo\":" + String(portOutputVoltage[i]) + ",\"PVset\":" + String(portVoltageSettings[i]) + ",";
			else portOutput[i] = portOutput[i] + "\"Vi\":null,\"Vo\":null,\"PVset\":null,";
		}
		if(talonPresent) sysOutput = sysOutput + "\"5V_BUS\":" + String(busVoltage_5V) + ","; 
		else sysOutput = sysOutput + "\"5V_BUS\":null,"; 

	}

	if(diagnosticLevel <= 5) {
		// output = output + "\"lvl-5\":{"; //OPEN JSON BLOB
		for(int i = 0; i < 3; i++) {
			overflow[i] = ioBeta.getInterrupt(pinsBeta::OVF1 + i); //Read in overflow values
			faults[i] = ioAlpha.getInterrupt(pinsAlpha::FAULT1 + i); //Read in fault values
			if (overflow[i] == true) {
				throwError(COUNTER_OVERFLOW | talonPortErrorCode | i + 1); //Throw overflow error with given port appended 
			}
			if (faults[i] == true) {
				throwError(AUX_POWER_FAIL | talonPortErrorCode | i + 1); //Throw power fault error with given port appended 
			}
		}

		// output = output + "\"ALPHA\":" + String(ioAlpha.readBus()) + ","; //Append ALPHA port readout
		// output = output + "\"BETA\":" + String(ioBeta.readBus()) + ","; //Append BETA port readout
		// output = output + "\"ALPHA_INT\":" + String(ioAlpha.getAllInterrupts(PCAL9535A::IntAge::BOTH)) + ","; //Append ALPHA interrupt readout
		// output = output + "\"BETA_INT\":" + String(ioBeta.getAllInterrupts(PCAL9535A::IntAge::BOTH)) + ","; //Append BETA interrupt readout
		// output = output + "\"PORT_CFG\":[" + String(portVoltageSettings[0]) + "," + String(portVoltageSettings[1]) + "," + String(portVoltageSettings[2]) + "],"; 
		if(talonPresent) sysOutput = sysOutput + "\"ALPHA\":" + String(ioAlpha.readBus()) + ",\"BETA\":" + String(ioBeta.readBus()) + ",";
		else sysOutput = sysOutput + "\"ALPHA\":null,\"BETA\":null,";
		if(talonPresent) {
			sysOutput = sysOutput + "\"I2C\":[";
			for(int adr = 0; adr < 128; adr++) { //Check for addresses present 
				Wire.beginTransmission(adr);
				// Wire.write(0x00);
				if(Wire.endTransmission() == 0) {
					sysOutput = sysOutput + String(adr) + ",";
				}
			}
			if(sysOutput.substring(sysOutput.length() - 1).equals(",")) {
				sysOutput = sysOutput.substring(0, sysOutput.length() - 1); //Trim trailing ',' is present
			}
			sysOutput = sysOutput + "],"; // close array
		}
		else sysOutput = sysOutput + "\"I2C\":[null],";
		// output = output + "}"; //CLOSE JSON BLOB, 
		ioAlpha.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Alpha
		ioBeta.clearInterrupt(PCAL9535A::IntAge::BOTH); //Clear all interrupts on Beta
		// return output + ",\"Pos\":[" + String(port) + "]}}";
		// return output;
		for(int i = 0; i < numPorts; i++) {
			if(talonPresent) portOutput[i] = portOutput[i] + "\"OVF\":" + String(overflow[i]) + ",\"Fault\":" + String(faults[i]);
			else portOutput[i] = portOutput[i] + "\"OVF\":null,\"Fault\":null";
		}
	}
	sysOutput = sysOutput + "\"Pos\":[" + getTalonPortString() + "]";
	for(int i = 0; i < numPorts; i++) {
		output = output + portOutput[i] + "},";
		// if(i < numPorts - 1) output = output + ","; //Only add comma if not last 
	}
	output = output + sysOutput + "}"; 
	if((millis() - diagnosticStart) > collectMax) throwError(EXCEED_COLLECT_TIME | 0x200 | talonPortErrorCode | sensorPortErrorCode); //Throw error for diagnostic taking too long
	return output;
	// return output + ",\"Pos\":[" + getTalonPortString() + "]}}"; //Write position in logical form - Return compleated closed output
	// else return ""; //Return empty string if reaches this point 

	// return "{}"; //Return null if reach end	
	// return output + ",\"Pos\":[" + String(port) + "]}}"; //Append position and return
}

int AuxTalon::restart()
{
	bool hasCriticalError = false;
	bool hasError = false;
	if(hasReset() && initDone) begin(0, hasCriticalError, hasError); //If for some reason the begin() function has not been run, call this now //FIX!
	setPinDefaults(); //Reset IO expander pins to their default state
	for(int i = 0; i < 3; i++) {
		if (faults[i] == true) { 
			if(ioAlpha.digitalRead(pinsAlpha::FAULT1 + i) == LOW) { //If the FAULT is still asserted 
				ioAlpha.digitalWrite(pinsAlpha::EN1 + i, HIGH); //Turn port power ON
				ioAlpha.digitalWrite(pinsAlpha::EN1 + i, LOW); //Turn port off
				ioAlpha.digitalWrite(pinsAlpha::EN1 + i, HIGH); //Turn port back on finally
				delay(10); //Wait for trip
				if(ioAlpha.digitalRead(pinsAlpha::FAULT1 + i) == LOW) { //If FAULT is re-asserted after power cycle
					throwError(AUX_POWER_FAIL_PERSISTENT | talonPortErrorCode | i + 1); //Throw persistent power fault error with given port appended 
				}
			}
		}
	}
	return 0; //FIX!
}

String AuxTalon::getData(time_t time)
{
	unsigned long dataStart = millis();
	const time_t startTime = clearTime; //Grab current clear time //FIX! change to report the time used in calculation
	const time_t stopTime = time; //Grab the time the current update is made
	String output = "\"Talon-Aux\":"; //OPEN JSON BLOB
	String portString[3] = {""};
	if(isPresent()) { //If Talon has been detected, go through normal data appending 
		updateCount(time); //Update counter values
		updateAnalog(); //Update analog readings
		output = output + "{"; //Open sub-blob
		// String analogData = "\"AIN\":[";
		// String analogAvgData = "\"AIN_AVG\":[";
		// String countData = "\"COUNTS\":[";
		// String rateData = "\"RATE\":[";

		for(int i = 0; i < 3; i++) {
			portString[i] = "\"PORT_" + String(i + 1) + "\":["; //Open blob
			portString[i] = portString[i] + String(analogVals[i], 7) + ",";
			portString[i] = portString[i] + String(analogValsAvg[i], 7) + ",";
			portString[i] = portString[i] + String(counts[i]) + ",";
			portString[i] = portString[i] + String(rates[i], 7);
			portString[i] = portString[i] + "]"; //Close blob
			// analogData = analogData + String(analogVals[i], 7) + ",";
			// analogAvgData = analogAvgData + String(analogValsAvg[i], 7) + ",";
			// countData = countData + String(counts[i]) + ",";
			// rateData = rateData + String(rates[i], 7) + ",";
		}
		// analogData = analogData.substring(0,analogData.length() - 1) + "],"; //Trim trailing ',' and close array
		// analogAvgData = analogAvgData.substring(0,analogAvgData.length() - 1) + "],";
		// countData = countData.substring(0,countData.length() - 1) + "],";
		// rateData = rateData.substring(0,rateData.length() - 1) + "],";

		// output = output + analogData + analogAvgData + countData + rateData; //Concatonate all sub-strings
		output = output + portString[0] + "," + portString[1] + "," + portString[2] + ","; //Concatonate each set of port values 
		output = output + "\"START\":" + String((long) startTime) + ","; //Concatonate start time
		output = output + "\"STOP\":" + String((long) stopTime) + ","; //Concatonate stop time
		output = output + "\"Pos\":[" + getTalonPortString() + "]"; //Concatonate position 
		output = output + "}"; //CLOSE JSON BLOB
	}
	else output = output + "null"; //Close with null
	if((millis() - dataStart) > collectMax) throwError(EXCEED_COLLECT_TIME | 0x100 | talonPortErrorCode | sensorPortErrorCode); //Throw error for data taking too long
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
			throwError(DEVICE_RESET | talonPortErrorCode); //Report the unplanned reset
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
	if(time > clearTime && (clearTime != 0 || initDone == false)) { //Make sure new time is logically consistent with previous reading time and clearing time, give pass to bad clearTime for first run  
		clearTime = time; //Swap clear time for read time  
		timeBaseGood = true; //FIX - Check in more aggresive way??
	}
	// else if(time < readTime || time < clearTime || time == 0) { 
	else if(time < clearTime || time == 0) { 	
		clearTime = 0;
		// readTime = 0; 
		timeBaseGood = false; //If any of time times are inconsistent, set the timebase to bad 
		throwError(TIME_BAD | talonPortErrorCode);
	}
	else if((time - clearTime) > maxTimeDelta && initDone == true) { //Only trigger if delta is exceeded after initial setup. On setup, time will go from 0 to correct time, this will trigger excess delta erroniously 
		clearTime = time; //Copy time value over 
		timeBaseGood = false; //Indicate that time base is not secure, but not confident in the failure 
		throwError(TIME_DELTA_EXCEEDED | talonPortErrorCode);
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
		throwError(AUX_ADC_READ_FAIL | (error << 8) | talonPortErrorCode); //Alert to error if I2C comunication problem 
		return 0; //Exit with error condition 
	}
	bool newRead = (readADCReg(0x01) >> 15) & 0x01; //Grab OS bit from config reg
	unsigned long localTime = millis();
	while(!newRead && (millis() - localTime) < adcReadTimeout) { //if there is not already a new read, continue to read from config reg until new read is reported or timeout occours 
		newRead = (readADCReg() >> 15) & 0x01 ;
	}
	if(!newRead) {
		throwError(ADC_TIMEOUT | talonPortErrorCode); //If new reading is not done, throw timeout error
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
		throwError(AUX_ADC_READ_FAIL | (error << 8) | talonPortErrorCode); //Alert to error if I2C comunication problem 
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
		throwError(AUX_ADC_READ_FAIL | (error << 8) | talonPortErrorCode); //If I2C timeout occoured, throw I2C error as well 
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
		throwError(AUX_ADC_READ_FAIL | (error << 8) | talonPortErrorCode); //If I2C timeout occoured, throw I2C error as well 
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

String AuxTalon::getMetadata()
{
	unsigned long metadataStart = millis();
	Wire.beginTransmission(0x58); //Write to UUID range of EEPROM
	Wire.write(0x98); //Point to start of UUID
	int error = Wire.endTransmission();
	// uint64_t uuid = 0;
	String uuid = "";

	if(error != 0) throwError(TALON_EEPROM_READ_FAIL | (error << 8) | talonPortErrorCode);
	else {
		uint8_t val = 0;
		Wire.requestFrom(0x58, 8); //EEPROM address
		for(int i = 0; i < 8; i++) {
			val = Wire.read();//FIX! Wait for result??
			// uuid = uuid | (val << (8 - i)); //Concatonate into full UUID
			uuid = uuid + String(val, HEX); //Print out each hex byte
			// Serial.print(Val, HEX); //Print each hex byte from left to right
			// if(i < 7) Serial.print('-'); //Print formatting chracter, don't print on last pass
			if(i < 7) uuid = uuid + "-"; //Print formatting chracter, don't print on last pass
		}
	}

	String metadata = "\"Talon-Aux\":{";
	if(error == 0) metadata = metadata + "\"SN\":\"" + uuid + "\","; //Append UUID only if read correctly, skip otherwise 
	metadata = metadata + "\"Hardware\":\"v" + String(version >> 4, HEX) + "." + String(version & 0x0F, HEX) + "\","; //Report hardware version as modded BCD
	metadata = metadata + "\"Firmware\":\"v" + FIRMWARE_VERSION + "\","; //Report firmware version as modded BCD
	metadata = metadata + "\"Pos\":[" + getTalonPortString() + "]"; //Concatonate position 
	metadata = metadata + "}"; //CLOSE  
	if((millis() - metadataStart) > collectMax) throwError(EXCEED_COLLECT_TIME | 0x300 | talonPortErrorCode | sensorPortErrorCode); //Throw error for metadata taking too long
	return metadata; 
}

// uint8_t AuxTalon::totalErrors()
// {
// 	return numErrors;
// }

// bool AuxTalon::ovfErrors()
// {
// 	if(numErrors > MAX_NUM_ERRORS) return true;
// 	else return false;
// }

// uint8_t AuxTalon::getTalonPort()
// {
// 	return port + 1; //Switch to rational counting
// }

// uint8_t AuxTalon::getTalon()
// {
// 	return port;
// }

// void AuxTalon::setTalonPort(uint8_t port_)
// {
// 	// if(port_ > numPorts || port_ == 0) throwError(PORT_RANGE_ERROR | portErrorCode); //If commanded value is out of range, throw error 
// 	if(port_ > 4 || port_ == 0) throwError(PORT_RANGE_ERROR | portErrorCode); //If commanded value is out of range, throw error //FIX! How to deal with magic number? This is the number of ports on KESTREL, how do we know that??
// 	else { //If in range, update the port values
// 		talonPort = port_ - 1; //Set global port value in index counting
// 		portErrorCode = (talonPort + 1) << 4; //Set port error code in rational counting 
// 	}
// }

int AuxTalon::sleep()
{
	//Turn off 5V if possible
	//Turn off analog ports 
	return 0; //DEBUG!
}

int AuxTalon::wake()
{
	return 0;
}

int AuxTalon::disableDataAll()
{
	for(int i = 1; i <= numPorts; i++) {
		enableData(i, false);
	}
	return 0; //DEBUG!
}

int AuxTalon::disablePowerAll()
{
	for(int i = 1; i <= numPorts; i++) {
		enablePower(i, false);
	}
	return 0; //DEBUG!
}

int AuxTalon::enableData(uint8_t port, bool state)
{
	return 0; //DEBUG!
}

int AuxTalon::enablePower(uint8_t port, bool state)
{
	return 0; //DEBUG!
}

bool AuxTalon::isPresent()
{
	//FIX! Update for more complete interrogation 
	Wire.beginTransmission(0x23);
	int error = Wire.endTransmission();
	if(error == 0) return true;
	else return false;
	// return false; //DEBUG!
}