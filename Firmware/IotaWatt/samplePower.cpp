#include "IotaWatt.h"
  
  /***************************************************************************************************
  *  samplePower()  Sample a channel.
  *  
  ****************************************************************************************************/
void samplePower(int channel, int overSample){
  static uint32_t trapTime = 0;
  uint32_t timeNow = millis();

  trace(T_POWER,0,channel);
  if( ! inputChannel[channel]->isActive()){
    return;
  }
  
      // If it's a voltage channel, use voltage only sample, update and return.

  trace(T_POWER,0);
  if(inputChannel[channel]->_type == channelTypeVoltage){
    float VRMS = sampleVoltage(channel, inputChannel[channel]->_calibration);
    if(VRMS >= 0.0){
      inputChannel[channel]->setVoltage(VRMS);                                                                        
    }
    return;
  }

         // Currently only voltage and power channels, so return if not one of those.
     
  if(inputChannel[channel]->_type != channelTypePower) return;

         // From here on, dealing with a power channel and associated voltage channel.

  trace(T_POWER,1);
  IotaInputChannel* Ichannel = inputChannel[channel];
  IotaInputChannel* Vchannel = inputChannel[Ichannel->_vchannel]; 
          
  byte Ichan = Ichannel->_channel;
  byte Vchan = Vchannel->_channel;
  
  double _Irms = 0;
  double _watts = 0;
  double _Vrms = 0;
  double _VA = 0;

  int16_t* VsamplePtr = Vsample;
  int16_t* IsamplePtr = Isample;
   
        // Invoke high speed sample collection.
        // If it fails, return.
 
  if(int rtc = sampleCycle(Vchannel, Ichannel)) {
    trace(T_POWER,2);
    if(rtc == 2){
      Ichannel->setPower(0.0, 0.0);
    }
    return;
  }          
      
        // Voltage calibration is the ratio of line voltage to voltage presented at the input.
        // Input voltage is further attenuated with voltage dividing resistors (Vadj_3).
        // So ratio of voltage at ADC vs line is calibration * Vadj_3.
    
  double Vratio = Vchannel->_calibration * Vadj_3 * getAref(Vchan) / double(ADC_RANGE);

        // Compute Vrms from raw samples

  _Vrms = Vratio * sqrt((double)sumVsq / samples);
  
        // Iratio is straight Amps/ADC volt.
  
  double Iratio = Ichannel->_calibration * getAref(Ichan) / double(ADC_RANGE);

        // Compute Irms from raw samples

  _Irms = Iratio * sqrt((double)sumIsq / samples);
  
      // Determine phase correction components.
      // stepCorrection is the number of V samples to add or subtract.
      // stepFraction is the interpolation to apply to the next V sample (0.0 - 1.0)
      // The phase correction is the net phase lead (+) of current computed as the 
      // (CT lead - VT lead) - any gross phase correction for 3 phase measurement.
      // Note that a reversed CT can be corrected by introducing a 180deg gross correction.

  float _phaseCorrection =  (Ichannel->getPhase(_Irms) - Vchannel->getPhase(_Vrms) - Ichannel->_vphase) * samples / 360.0;  // fractional Isamples correction
  int stepCorrection = int(_phaseCorrection);                                        // whole steps to correct 
  float stepFraction = _phaseCorrection - stepCorrection;                            // fractional step correction
  if(stepFraction < 0){                                                              // if current lead
    stepCorrection--;                                                                // One sample back
    stepFraction += 1.0;                                                             // and forward 1-fraction
  }
  Ichannel->_lastPhase = Ichannel->getPhase(_Irms) - Vchannel->getPhase(_Vrms);

  trace(T_POWER,3);

        // Recompute sums and squares with phase corrected samples.

  int16_t rawV;
  int16_t rawI;  
  double _sumVI = 0;
  double _sumVsq = 0;
  double _sumIsq = 0;
            
  Isample[samples] = Isample[0];
  Vsample[samples] = Vsample[0];      
  int Vindex = (samples + stepCorrection) % samples;
  for(int i=0; i<samples; i++){  
    rawI = *IsamplePtr;
    rawV = Vsample[Vindex]; 
    rawV += int(stepFraction * (Vsample[Vindex + 1] - Vsample[Vindex]));
    _sumVsq += rawV * rawV;
    _sumIsq += rawI * rawI;
    _sumVI += rawV * rawI;      
    IsamplePtr++;
    Vindex = ++Vindex % samples;
  }

        // Compute Vrms, Irms, Power, etc.

  _Vrms = Vratio * sqrt((double)_sumVsq / samples);
  _Irms = Iratio * sqrt((double)_sumIsq / samples);
  _watts = Vratio * Iratio * ((double)_sumVI / samples);
  _VA = _Vrms * _Irms;

  _watts *= Ichannel->_vmult;
  _VA *= Ichannel->_vmult;
  
        // If watts is negative and the channel is not explicitely signed, reverse it (backward CT).
        // If we do reverse it, and it's significant, mark it as such for reporting in the status API.

  Ichannel->_reversed = false;
  if( ! Ichannel->_signed){
    if(_watts < 0){
      _watts = -_watts;
      if(_watts > 5){
        Ichannel->_reversed = true;
      }
    }
  }
      // Update with the new power and voltage values.

  trace(T_POWER,5);
  Ichannel->setPower(_watts, _VA);
  trace(T_POWER,9);                                                                               
  return;
}

//**********************************************************************************************
//
//        readADC(uint8_t channel)
//
//**********************************************************************************************

int readADC(uint8_t channel){
  uint16_t ADC_in = 0xFFFF;
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // SD may have changed this
  while (((ADC_in & 0xF000) >> 12) != (inputChannel[channel]->_addr & 0xF))
  {
    ADC_in = readADCsingle(channel);
  }
  return (ADC_in & 0xFFF); // Put the result together and return
}

//**********************************************************************************************
//
//        readADCsequence(uint8_t channel)
//        Prepares read that in a known order
//**********************************************************************************************

int readADCsequence(uint8_t channel, uint8_t nextChannel){

  uint16_t ADC_in = 0xFFFF; // Invalidate for first read
  
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // SD may have changed this

  while (((ADC_in & 0xF000) >> 12) != (inputChannel[channel]->_addr & 0xF))
  {
    ADC_in = readADCsingle(channel);
    if (((ADC_in & 0xF000) >> 12) == (inputChannel[channel]->_addr & 0xF))
    { // Now that we have the necessary first read, prepare the sequence
      readADCsingle(nextChannel);
      readADCsingle(channel);
    }
  }

  return (ADC_in); // Put the result together and return
}


//**********************************************************************************************
//
//        readADCsingle(uint8_t channel)
//
//**********************************************************************************************

uint16_t readADCsingle(uint8_t channel){
  //Serial.printf_P(PSTR("readADC chan %d\r\n"), channel);
  constexpr uint32_t dataMask = ((ADC_BITS + 3) << SPILMOSI) | ((ADC_BITS + 3) << SPILMISO);
  constexpr uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
  const uint16_t config = ( 0b0001 << 12 ) |         // Manual read of configured channel
                              ( 1 << 11 ) |              // Enable setting of configuration functions
                              ( (inputChannel[channel]->_addr & 0xF) << 7 ) |
                              ( 1 << 6 );                // 2xVref input range

  volatile uint8_t * fifoPtr8 = (volatile uint8_t *) &SPI1W0;

  uint8_t ADCselectPin = ADC_selectPin[inputChannel[channel]->_addr >> 4]; // Breaks down the channel to 0 or 1
  uint32_t ADC_selectMask = 1 << ADCselectPin;             // Mask for hardware chip select (pins 0-15)

  if (ADCselectPin == 16) GP16O &= ~1;
  else                    GPOC = ADC_selectMask;                // Lower the chip select

  SPI1U1 = (SPI1U1 & mask) | dataMask;                          // Set number of bits
  SPI1W0 = ((config >> 8) & 0xFF) | ((config & 0xFF) << 8);     // Manual request to ADC
  SPI1CMD |= SPIBUSY;                                           // Start the SPI clock  
  while(SPI1CMD & SPIBUSY) {}                                   // Loop till SPI completes

  if (ADCselectPin == 16) GP16O |= 1;
  else                    GPOS = ADC_selectMask;                // Raise the chip select to deselect and reset

  return word(*fifoPtr8, *(fifoPtr8+1)); // Put the result together and return
}

/****************************************************************************************************
 * sampleVoltage() is used to sample just voltage and is also used by the voltage calibration handler.
 * It uses sampleCycle specifying the voltage channel for both channel parameters thus 
 * doubling the number of voltage samples.
 * It returns the voltage corresponding to the supplied calibration factor
 ****************************************************************************************************/
float sampleVoltage(uint8_t Vchan, float Vcal){
  IotaInputChannel* Vchannel = inputChannel[Vchan];
  uint32_t sumVsq = 0;
  int retries = 0;
  while(int rtc = sampleCycle(Vchannel, Vchannel)){
    if(rtc == 2){
      return 0.0;
    }
    if(retries ++ > 3){
      return -1.0;
    }
  }
  for(int i=0; i<samples; i++){  
    sumVsq += Vsample[i] * Vsample[i];
    sumVsq += Isample[i] * Isample[i];
  }
  double Vratio = Vcal * Vadj_3 * getAref(Vchan) / double(ADC_RANGE);
  return  Vratio * sqrt((double)(sumVsq / (samples * 2)));
}
//**********************************************************************************************
//
//        getAref()  -  Get the current value of Aref
//
//**********************************************************************************************

float getAref(int channel) {
  uint32_t dataMask = ((ADC_BITS + 3) << SPILMOSI) | ((ADC_BITS + 3) << SPILMISO);
  constexpr uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
  const uint16_t config = ( 0b0001 << 12 ) |         // Manual read of configured channel
                    ( 1 << 11 ) |              // Enable setting of configuration functions
                    ( (inputChannel[channel]->_aRef & 0xF) << 7) |       // Channel to request
                    ( 1 << 6 );                // 2xVref input range
  uint8_t ADCselectPin = ADC_selectPin[inputChannel[channel]->_aRef >> 4];
  uint32_t ADC_selectMask = 1 << ADCselectPin;             // Mask for hardware chip select (pins 0-15)
  uint16_t ADC_in = 0xFFFF; // Invalidate for first read
  volatile uint8_t * fifoPtr8 = (volatile uint8_t *) &SPI1W0;
  
  SPI.beginTransaction(SPISettings(10000000,MSBFIRST,SPI_MODE0));  // SD may have changed this

  while (((ADC_in & 0xF000) >> 12) != (inputChannel[channel]->_aRef & 0xF))
  {
    if (ADCselectPin == 16)   GP16O &= ~1;
    else                      GPOC = ADC_selectMask;                                     //digitalWrite(ADCselectPin, LOW);                  // Lower the chip select
    SPI1U1 = (SPI1U1 & mask) | dataMask;                          // Set number of bits
    SPI1W0 = ((config >> 8) & 0xFF) | ((config & 0xFF) << 8);     // Manual request to ADC
    SPI1CMD |= SPIBUSY;                                           // Start the SPI clock  
    while(SPI1CMD & SPIBUSY) {}                                   // Loop till SPI completes
    if (ADCselectPin == 16) GP16O |= 1;
    else                    GPOS = ADC_selectMask;                                     //digitalWrite(ADCselectPin, HIGH);                 // Raise the chip select to deselect and reset
    ADC_in = word(*fifoPtr8, *(fifoPtr8+1));
  }
  uint16_t ADCvalue = (ADC_in & 0xFFF);
  if(ADCvalue == 4095 | ADCvalue == 0) return 0;    // no ADC
  return VrefVolts * ADC_RANGE / ADCvalue;  
}

//**********************************************************************************************
//
//        printSamples()  -  print the current samples.
//        These are diagnostic tools, not currently used.
//
//**********************************************************************************************

void printSamples() {
  Serial.println(samples);
  for(int i=0; i<(samples + 2); i++)
  {
    Serial.print(i);
    Serial.print(", ");
    Serial.print(Vsample[i]);
    Serial.print(", ");
    Serial.print(Isample[i]);
    Serial.println();
  }
  return;
}

// String samplePhase(uint8_t Vchan, uint8_t Ichan, uint16_t shift){

//   trace(T_samplePhase,0);
//   int cycles = 10;
//   int Ishift = shift;

//   IotaInputChannel* Vchannel = inputChannel[Vchan]; 
//   IotaInputChannel* Ichannel = inputChannel[Ichan];
  
//   uint32_t dataMask = ((ADC_BITS + 6) << SPILMOSI) | ((ADC_BITS + 6) << SPILMISO);
//   const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
//   volatile uint8_t * fifoPtr8 = (volatile uint8_t *) &SPI1W0;
  
//   uint8_t  Iport = inputChannel[Ichan]->_addr % 8;       // Port on ADC
//   uint8_t  Vport = inputChannel[Vchan]->_addr % 8;
    
//   int16_t offsetV = Vchannel->_offset;        // Bias offset
//   int16_t offsetI = Ichannel->_offset;
  
//   int16_t rawV;                               // Raw ADC readings
//   int16_t lastV;
//   int16_t avgV;
//   int16_t rawI;
//   int16_t *Vsamples = new int16_t[Ishift+1];
            
//   int16_t crossLimit = cycles * 2 + 1;        // number of crossings in total
//   int16_t crossCount = 0;                     // number of crossings encountered
//   int16_t crossGuard = 3;                     // Guard against faux crossings (must be >= 2 initially)  

//   uint32_t startMs = millis();                // Start of current half cycle
//   uint32_t timeoutMs = 12;                    // Maximum time allowed per half cycle
//   uint32_t firstCrossUs;                      // Time cycle at usec resolution for phase calculation
//   uint32_t lastCrossUs;                       

//   byte ADC_IselectPin = ADC_selectPin[inputChannel[Ichan]->_addr >> 3];  // Chip select pin
//   byte ADC_VselectPin = ADC_selectPin[inputChannel[Vchan]->_addr >> 3];
//   uint32_t ADC_IselectMask = 1 << ADC_IselectPin;             // Mask for hardware chip select (pins 0-15)
//   uint32_t ADC_VselectMask = 1 << ADC_VselectPin;

//   bool Vreverse = inputChannel[Vchan]->_reverse;
//   bool Ireverse = inputChannel[Ichan]->_reverse;

//   double sumVsq = 0.0;
//   double sumIsq = 0.0;
//   double sumVI  = 0.0;
//   uint32_t samples = 0;

//   SPI.beginTransaction(SPISettings(2000000,MSBFIRST,SPI_MODE0));
 
//   rawV = readADC(Vchan) - offsetV;                    // Prime the pump
//   samples = 0;                                        // Start with nothing

//           // Have at it.

//   trace(T_samplePhase,1);
//   ESP.wdtFeed();                                     // Red meat for the silicon dog
//   WDT_FEED();
//   trace(T_samplePhase,1);   
//   do{  
//                       //************************************
//                       //* Sample the Current (I) channel   *
//                       //************************************
                                               
//         GPOC = ADC_IselectMask;                            // digitalWrite(ADC_IselectPin, LOW); Select the ADC
//         SPI1U1 = (SPI1U1 & mask) | dataMask;               // Set number of bits 
//         SPI1W0 = (0x18 | Iport) << 3;                      // Data left aligned in low byte 
//         SPI1CMD |= SPIBUSY;                                // Start the SPI clock 

//               // Do some loop housekeeping asynchronously while SPI runs.

//           if(crossCount) {                                  // If past first crossing
//             int32_t V;
//             if(shift){
//               int Vndx = samples % Ishift;
//               V = Vsample[Vndx];
//               Vsample[Vndx] = (rawV + lastV) >> 1;
//             } else {
//               V = (rawV + lastV) >> 1; 
//             } 
//             if(samples >= Ishift){
//               sumIsq += rawI * rawI;
//               sumVsq += V * V;
//               sumVI += rawI * V;
//             } 
//             samples++;
//           }
//           lastV = rawV;
//           crossGuard--;    
          
//               // Now wait for SPI to complete
        
//         while(SPI1CMD & SPIBUSY) {}                                         // Loop till SPI completes
//         GPOS = ADC_IselectMask;                                             // digitalWrite(ADC_IselectPin, HIGH); Deselect the ADC 
//         rawI = (word(*fifoPtr8 & 0x01, *(fifoPtr8+1)) << 3) + (*(fifoPtr8+2) >> 5) - offsetI;
//         if(Ireverse) rawI = -rawI;

//                       //************************************
//                       //*  Sample the Voltage (V) channel  *
//                       //************************************
         
//         GPOC = ADC_VselectMask;                             // digitalWrite(ADC_VselectPin, LOW); Select the ADC
//         SPI1U1 = (SPI1U1 & mask) | dataMask;
//         SPI1W0 = (0x18 | Vport) << 3;
//         SPI1CMD |= SPIBUSY;
        
//               // Do some housekeeping asynchronously while SPI runs.
//               // Check for timeout.  The clock gets reset at each crossing, so the
//               // timeout value is a little more than a half cycle - 10ms @ 60Hz, 12ms @ 50Hz.
//               // The most common cause of timeout here is unplugging the AC reference VT.  Since the
//               // device is typically sampling 60% of the time, there is a high probability this
//               // will happen if the adapter is unplugged.
//               // So handling needs to be robust.
        
//           if((uint32_t)(millis()-startMs)>timeoutMs){                   // Something is wrong
//             trace(T_SAMP,2);                                            // Leave a meaningful trace
//             trace(T_SAMP,Ichan);
//             trace(T_SAMP,Vchan);
//             GPOS = ADC_VselectMask;                                     // ADC select pin high 
//             delete[] Vsamples;                               
//             return "Sample Timeout";                                                // Return a failure
//           }
                              
//               // Now wait for SPI to complete
        
//         while(SPI1CMD & SPIBUSY) {}                                 
//         GPOS = ADC_VselectMask;                           // digitalWrite(ADC_VselectPin, HIGH);  Deselect the ADC                       
//         rawV = (word(*fifoPtr8 & 0x01, *(fifoPtr8+1)) << 3) + (*(fifoPtr8+2) >> 5) - offsetV;
//         if(Vreverse) rawV = -rawV;

//         // Finish up loop cycle by checking for zero crossing.
//         // Crossing is defined by voltage changing signs  (Xor) and crossGuard negative.

//         if(((rawV ^ lastV) & crossGuard) >> 15) {        // If crossed unambiguously (one but not both Vs negative and crossGuard negative 
//           startMs = millis();                            // Reset the cycle clock 
//           crossCount++;
//           crossGuard = 10;                              // No more crosses for awhile                                    // Count the crossings 
//           if(crossCount == 1){
//             trace(T_SAMP,4);
//             rawV = rawV >> 1;
//             samples = 0;
//             firstCrossUs = micros();
//           }
//           else if(crossCount == crossLimit) {
//             trace(T_SAMP,6);
//             rawV = rawV >> 1;
//             lastCrossUs = micros();                       // To compute frequency
//             crossGuard = Ishift + 1;                      // Finish sampling shifted I
//           }
//         }
//   } while(crossCount < crossLimit || crossGuard > 0);

//   trace(T_samplePhase,2);
//   trace(T_samplePhase,3);

//   delete[] Vsamples;
//   samples -= (Ishift + 1);
//   double Vrms = sqrt(sumVsq / samples); 
//   double Irms = sqrt(sumIsq / samples);
//   double VI = sumVI / samples;
//   float  phaseDiff = ((double)57.29578 * acos(VI / (Vrms * Irms))) - 0.045;
//   double IshiftDeg = (double)Ishift * (360.0 * (float)cycles) / (float)samples;

//   xbuf response;
//   response.printf_P(PSTR("Sample phase lead\r\nChannel: %d\r\n"), Ichan) ;
//   response.printf_P(PSTR("samples: %d, sample degrees: %.3f\r\n"), samples, 360.0 * (float)cycles/(float)samples);
//   //response.printf_P(PSTR("Ius: %d, Vus %d\r\n"), Ius, Vus);
//   //response.printf_P(PSTR("lV: %d, rV: %d, firstV: %d, firstI: %d\r\n\r\n"), lV, rV, firstV, firstI);
//   response.printf_P(PSTR("Measured shift: %.2f degrees\r\n"), phaseDiff);
//   response.printf_P(PSTR("Artificial shift: %.2f degrees (%d) samples\r\n"), IshiftDeg, Ishift);
//   response.printf_P(PSTR("Net shift: %.2f degrees\r\n"), phaseDiff-IshiftDeg);
    
//   return response.readString(); 
// }

//**********************************************************************************************
//
//        samplePhase()  -  Measure phase between two signals
//  
//        Returns:  Phase shift measured in degrees
//        Also:     IPri = Primary current measured
//                  ISec = CT Secondary current measured
//
//        Initializes by sampling for a few cycles to get the average Ichan ADC
//        value to be used as offset.  offset precision is increased by using a
//        14 bit value.  ADC readings are padded with two zeroes.
//
//        Once offset is determined, the main loop runs for a lot of cycles
//        to accumulate data for phase measurement, Irms, Crms etc.
//        These measurements are processed into double accumulators
//        As they are taken.  There is a circular buffer for "shift"
//        Isamples to do the sample shift.
//
//        In the end, the net shift is returned along with the adc voltages
//        of the Ichan and Cchan.
//
//**********************************************************************************************
float samplePhase(uint8_t Ichan, uint8_t Cchan, int shift){
  double VPri, VSec;
  return samplePhase(Ichan, Cchan, shift, &VPri, &VSec);
}

float samplePhase(uint8_t Ichan, uint8_t Cchan, int shift, double *VPri, double *VSec){

  int offsetCycles = 5;                            // Cycles sampled to determine offset value
  int cycles = 20;                                // Cycles sampled for phase calculation

  uint32_t dataMask = ((ADC_BITS + 3) << SPILMOSI) | ((ADC_BITS + 3) << SPILMISO);
  constexpr uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
  volatile uint8_t * fifoPtr8 = (volatile uint8_t *) &SPI1W0;

  IotaInputChannel* Ichannel = inputChannel[Ichan];
  IotaInputChannel* Cchannel = inputChannel[Cchan];

  byte ADC_IselectPin = ADC_selectPin[Ichannel->_addr >> 4];  // Chip select pin
  byte ADC_CselectPin = ADC_selectPin[Cchannel->_addr >> 4]; 
  
  uint32_t ADC_IselectMask = 1 << ADC_IselectPin;             // Mask for hardware chip select (pins 0-15)
  uint32_t ADC_CselectMask = 1 << ADC_CselectPin;
              
  uint8_t  Iport = inputChannel[Ichan]->_addr % 16;             // Port on ADC
  uint8_t  Cport = inputChannel[Cchan]->_addr % 16;         

  constexpr uint16_t adc_config = ( 0b0001 << 12 ) |         // Manual read of configured channel
                                  ( 1 << 11 ) |              // Enable setting of configuration functions
                                  ( 1 << 6 );                // 2xVref input range

  int16_t offsetI = 8192;       
  int16_t offsetC = 8192;
   
  int16_t rawI;                               // Raw ADC readings
  int16_t lastI;
  int16_t avgI;
  int16_t rawC;
  int16_t *isamples = new int16_t[shift+1];
        
  int16_t crossLimit = cycles * 2 + 1;        // number of crossings in total
  int16_t crossCount = 0;                     // number of crossings encountered
  int16_t crossGuard = 5;                     // Guard against faux crossings (must be >= 2 initially)  

  uint32_t startMs = millis();                // Start of current half cycle
  uint32_t timeoutMs = 15;                    // Maximum time allowed per half cycle
  uint32_t firstCrossUs;                      // Time cycle at usec resolution for phase calculation
  uint32_t lastCrossUs;                       

  bool Ireverse = inputChannel[Ichan]->_reverse;
  bool Creverse = inputChannel[Cchan]->_reverse;

  double sumIsq = 0.0;
  double sumCsq = 0.0;
  double sumIC  = 0.0;
  int32_t sumI = 0;
  int32_t sumC = 0;
  int32_t samples = 0;

  int32_t Iwait = 0;
  int32_t Cwait = 0;

  SPI.beginTransaction(SPISettings(10000000,MSBFIRST,SPI_MODE0));

  
  if (ADC_IselectPin == ADC_CselectPin)
  {
    rawI = readADCsequence(Ichan, Cchan) - offsetI;                    // Prime the pump
    //Serial.printf_P(PSTR("AD Sequence: %d %d\r\n"), Ichan, Cchan);
  }
  else
  { 
    rawI = (readADC(Ichan)) - offsetI;                    // Prime the pump
    readADC(Cchan); //Prime other ADC
    //Serial.printf_P(PSTR("AD No Seq: %d %d\r\n"), Ichan, Cchan);
  }
  samples = 0;

          // Sample to get offsets

  ESP.wdtFeed();                                     // Red meat for the silicon dog
  WDT_FEED();

  do{  
                      //*******************************
                      //* Sample the CT (C) channel   *
                      //*******************************
        uint16_t payload = adc_config | ( Cport << 7);
        
        if (ADC_CselectPin == 16)   GP16O &= ~1;
        else                        GPOC = ADC_CselectMask;                            // digitalWrite(ADC_CselectPin, LOW); Select the ADC

        SPI1U1 = (SPI1U1 & mask) | dataMask;               // Set number of bits 
        SPI1W0 = ((payload >> 8) & 0xFF) | ((payload & 0xFF) << 8);               // Manual request to ADC
        SPI1CMD |= SPIBUSY;                                // Start the SPI clock 

          if(crossCount) {
            sumI += (rawI + lastI) >> 1;
            sumC += rawC;                                  // If past first crossing
            samples++;
          }
          lastI = rawI;
          crossGuard--;    
          
        while(SPI1CMD & SPIBUSY) {}                                         // Loop till SPI completes
        if (ADC_CselectPin == 16) GP16O |= 1;
        else                      GPOS = ADC_CselectMask;                                             // digitalWrite(ADC_CselectPin, HIGH); Deselect the ADC 

        if ((*fifoPtr8 >> 4) != Cport)
        {
          Serial.printf_P(PSTR("Bad AD C sequence.  Needed %d, got %d\r\n"), Cport, (*fifoPtr8 >> 4));
          rawI = readADC(Cchan) - offsetC;
        }
        else
          rawC = (word(*fifoPtr8, *(fifoPtr8+1)) & 0xFFF) - offsetC;
        
        if(Creverse) rawC = -rawC;

                      //************************************
                      //*  Sample the Current (I) channel  *
                      //************************************
        payload = adc_config | ( Iport << 7 );
        if (ADC_IselectPin == 16)   GP16O &= ~1;
        else                        GPOC = ADC_IselectMask;                            // digitalWrite(ADC_IselectPin, LOW); Select the ADC

        SPI1U1 = (SPI1U1 & mask) | dataMask;
        SPI1W0 = ((payload >> 8) & 0xFF) | ((payload & 0xFF) << 8);               // Manual request to ADC
        SPI1CMD |= SPIBUSY;

          if((uint32_t)(millis()-startMs) > timeoutMs){                   // Something is wrong
            digitalWrite(ADC_IselectPin, HIGH);                           // Deselect the ADC
            delete[] isamples;                               
            Serial.printf("crosscount %d, samples %d\r\n", crossCount, samples);                                              
            return -998.0;  
          }
        
        while(SPI1CMD & SPIBUSY) {}                                 
        if (ADC_IselectPin == 16) GP16O |= 1;
        else                      GPOS = ADC_IselectMask;                                             // digitalWrite(ADC_IselectPin, HIGH); Deselect the ADC                  

        if ((*fifoPtr8 >> 4) != Iport)
        {
          Serial.printf_P(PSTR("Bad AD I sequence.  Needed %d, got %d\r\n"), Iport, (*fifoPtr8 >> 4));
          rawI = readADC(Ichan) - offsetI;
        }
        else
          // extract the rawI from the SPI hardware buffer and adjust with offset. 
          rawI = (word(*fifoPtr8, *(fifoPtr8+1)) & 0xFFF) - offsetI;
        
        if(Ireverse) rawI = -rawI;

        // Finish up loop cycle by checking for zero crossing.
        // Crossing is defined by voltage changing signs  (Xor) and crossGuard negative.

        if(((rawI ^ lastI) & crossGuard) >> 15) {        // If crossed unambiguously (one but not both Vs negative and crossGuard negative 
          startMs = millis();                            // Reset the cycle clock 
          crossCount++;
          crossGuard = 10;                               // No more crosses for awhile                                    // Count the crossings 
          if(crossCount == 1){
            rawI = rawI >> 1;
            samples = 0;
          }
          else if(crossCount == offsetCycles) {
            rawI = rawI >> 1;
            sumI += (rawI + lastI) >> 1;
            sumC += rawC;                                  
            samples++;                       
            break;
          }
        }
  } while(true);

  ESP.wdtFeed();                                          // Red meat for the silicon dog
  WDT_FEED();

  offsetI += sumI / samples;
  offsetC = offsetI;
  if (ADC_IselectPin == ADC_CselectPin)
  {
    rawI = readADCsequence(Ichan, Cchan) - offsetI;       // Prime the pump
    //Serial.printf_P(PSTR("AD Sequence: %d %d\r\n"), Ichan, Cchan);
  } else
  { 
    rawI = (readADC(Ichan)) - offsetI;                    // Prime the pump
    readADC(Cchan);                                       //Prime other ADC
    //Serial.printf_P(PSTR("AD No Seq: %d %d\r\n"), Ichan, Cchan);
  }
  sumI = sumC = samples = 0;
  crossCount = 0;
  crossGuard = 10;

  do{  

                      //*******************************
                      //* Sample the CT (C) channel   *
                      //*******************************
        uint16_t payload = (adc_config | ( Cport << 7));

        if (ADC_CselectPin == 16)   GP16O &= ~1;
        else                        GPOC = ADC_CselectMask;                             // digitalWrite(ADC_CselectPin, LOW); Select the ADC

        SPI1U1 = (SPI1U1 & mask) | dataMask;               // Set number of bits 
        SPI1W0 = ((payload >> 8) & 0xFF) | ((payload & 0xFF) << 8);               // Manual request to ADC
        SPI1CMD |= SPIBUSY;                                // Start the SPI clock 

              // Do some loop housekeeping asynchronously while SPI runs.

          if(crossCount) {                                  // If past first crossing
            avgI = (rawI + lastI) >> 1;
            int32_t I = avgI;
            if(shift){
              int Indx = samples % shift;
              I = isamples[Indx];
              isamples[Indx] = avgI;
            } 
            if(samples >= shift){
              sumI += I;
              sumC += rawC;
              sumCsq += rawC * rawC;
              sumIsq += I * I;
              sumIC += I * rawC;
            }
            samples++;
          }
          lastI = rawI;
          crossGuard--;    
          
              // Now wait for SPI to complete
        
        while(SPI1CMD & SPIBUSY) {Cwait++;}                                         // Loop till SPI completes
        if (ADC_CselectPin == 16)   GP16O |= 1;
        else                        GPOS = ADC_CselectMask;                           // digitalWrite(ADC_VselectPin, HIGH);  Deselect the ADC             
        if ((*fifoPtr8 >> 4) != Cport)
        {
          Serial.printf_P(PSTR("Bad AD C sequence.  Needed %d, got %d\r\n"), Cport, (*fifoPtr8 >> 4));
          rawC = readADC(Cchan) - offsetC;
        }
        else // extract the rawI from the SPI hardware buffer and adjust with offset.
          rawC = (word(*fifoPtr8, *(fifoPtr8+1)) & 0xFFF) - offsetC;
        if(Creverse) rawC = -rawC;

                      //************************************
                      //*  Sample the Current (I) channel  *
                      //************************************
        payload = (adc_config | ( Iport << 7));

        if (ADC_IselectPin == 16)   GP16O &= ~1;
        else                        GPOC = ADC_IselectMask;                             // digitalWrite(ADC_CselectPin, LOW); Select the ADC
        SPI1U1 = (SPI1U1 & mask) | dataMask;
        SPI1W0 = ((payload >> 8) & 0xFF) | ((payload & 0xFF) << 8);               // Manual request to ADC
        SPI1CMD |= SPIBUSY;
        
              // Do some housekeeping asynchronously while SPI runs.
              // Check for timeout.  The clock gets reset at each crossing, so the
              // timeout value is a little more than a half cycle - 10ms @ 60Hz, 12ms @ 50Hz.
              // The most common cause of timeout here is unplugging the AC reference VT.  Since the
              // deICce is typically sampling 60% of the time, there is a high probability this
              // will happen if the adapter is unplugged.
              // So handling needs to be robust.
        
          if((uint32_t)(millis()-startMs) > timeoutMs){                   // Something is wrong
            if (ADC_IselectPin == 16) GP16O |= 1;
            else                      GPOS = ADC_IselectMask;                                             // digitalWrite(ADC_IselectPin, HIGH); Deselect the ADC 
            delete[] isamples;                               
            return -999.0;                                                // Return a failure
          }
                              
              // Now wait for SPI to complete
        
        while(SPI1CMD & SPIBUSY) {Iwait++;}                                 
        if (ADC_IselectPin == 16) GP16O |= 1;
        else                      GPOS = ADC_IselectMask;                                             // digitalWrite(ADC_IselectPin, HIGH); Deselect the ADC                     

        if ((*fifoPtr8 >> 4) != Iport)
        {
          Serial.printf_P(PSTR("Bad AD I sequence.  Needed %d, got %d\r\n"), Iport, (*fifoPtr8 >> 4));
          rawI = readADC(Ichan) - offsetI;
        }
        else
          // extract the rawI from the SPI hardware buffer and adjust with offset. 
          rawI = (word(*fifoPtr8, *(fifoPtr8+1)) & 0xFFF) - offsetI;
        if(Ireverse) rawI = -rawI;

        // Finish up loop cycle by checking for zero crossing.
        // Crossing is defined by voltage changing signs  (Xor) and crossGuard negative.

        if(((rawI ^ lastI) & crossGuard) >> 15) {        // If crossed unambiguously (one but not both Vs negative and crossGuard negative 
          startMs = millis();                            // Reset the cycle clock 
          crossCount++;                                  // Count the crossings 
          crossGuard = 40;                               // No more crosses for awhile                                    
          if(crossCount == 1){
            rawI = rawI >> 1;
            samples = 0;
            firstCrossUs = micros();
          }
          else if(crossCount == crossLimit) {
            rawI = rawI >> 1;
            lastCrossUs = micros();                      // To compute frequency
            crossGuard = shift + 1;                      // Finish sampling shifted I
          }
        }
  } while(crossCount < crossLimit || crossGuard > 0);
  
  delete[] isamples;
  samples -= (shift + 1);
  double Irms = sqrt(sumIsq / samples); 
  double Crms = sqrt(sumCsq / samples);
  double IC = sumIC / samples;
  float  phaseDiff = ((double)57.29578 * acos(IC / (Irms * Crms))) - 0.040;
  double shiftDeg = (double)shift * (360.0 * (float)cycles) / (float)samples;
  *VPri = Irms * getAref(0) / 16384.0;
  *VSec = Crms * getAref(0) / 16384.0;

  return phaseDiff-shiftDeg; 
} 