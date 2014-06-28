// Current Issues: No way to detect changes in standard deviations yet. Will try to imlement
// Inclan&Tao's iterated cumulative sum of squares method.

// Sparkfun Spectrum Shield Driver v0.2 (contributors: Tom Flock, John Boxall, Elijah Gregory)
// Written using Arduino IDE v1.0.5 and Processing v2.0.3 on Nov 3rd, 2013. by Tom Flock
// Based on Example 48.1 - tronixstuff.com/tutorials > chapter 48 - 30 Jan 2013 by John Boxall
// Running stats computations added June 25th, 2014 by Elijah J. Gregory

// This code receives multiplexed data from the Spectrum Shield's two MSGEQ7 ICs
// and transmits the values via serial at a 115200 baud rate.
// The fourteen values are seperated by commas and are terminated by a newline,
// which is the format your visualization code should expect.

// Currently the driver computes the running average, variance, and standard deviation for each
// input band. Upon initialization the mean is computed assuming zero input volume. This mean is
// used to baseline correct later values. The driver outputs the average values to create smooth
// visualization input. The standard deviation is used to detect changes in the process generating
// the input data and will be used to detect visualization start/stop times.

// MSGEQ7 Control
int strobe = 4; // strobe pins on digital 4
int res = 5; // reset pins on digital 5
static const byte smoothP = 64;  // Number of samples to compute rolling average over (empirically set)
static const boolean _INIT_ = true;        // increasing 'smoothP' makes bars jutter less, but also increases <S-<S>> = c 
                                 
int left[7];                                     // store raw band values in these arrays
int right[7];
int averageR[7] = { 0, 0, 0, 0, 0, 0, 0};   // 'smoothP' running average of raw data 
int averageL[7] = { 0, 0, 0, 0, 0, 0, 0};
int _zeroBSLR[7] = { 0, 0, 0, 0, 0, 0, 0};       // bands with zero volume input
int _zeroBSLL[7] = { 0, 0, 0, 0, 0, 0, 0};
uint32_t stdDevL[7] = { 0, 0, 0, 0, 0, 0, 0};       // standard deviation of band inputs for change-point detection
uint32_t stdDevR[7] = { 0, 0, 0, 0, 0, 0, 0};
uint32_t varianceL[7] = { 0, 0, 0, 0, 0, 0, 0};  // unsigned 32-bit integer to store (1023)*(1023) maximum
uint32_t varianceR[7] = { 0, 0, 0, 0, 0, 0, 0};
uint32_t initValueL = 0;
uint32_t initValueR = 0;
uint32_t tmpVarL = 0;
uint32_t tmpVarR = 0;
int tmpAvgL = 0;
int tmpAvgR = 0;
int band;                                        // counting variable for going through channels

inline void reduce(int &anInt, int aAmount, int aMin = 0)
{
  int r = anInt-aAmount;
  if (r<aMin)
    anInt = aMin;
  else
    anInt = r;
}

inline void increase(int &anInt, int aAmount, int aMax = 1023)
{
  int r = anInt+aAmount;
  if (r>aMax)
    anInt = aMax;
  else
    anInt = r;
}

uint32_t findSqRoot(uint32_t aVariance) {
  uint32_t result = 0;              
  uint32_t var = aVariance;
  uint32_t check_bit = 1;             
  check_bit <<= 30;
  while (check_bit > var) {         
    check_bit >>= 2;                       
  }
  while (check_bit != 0) {              
    if (var >= (result + check_bit)) {   
      var -= (result + check_bit);       
      result = (result>>1) + check_bit;  
    } else {                             
      result >>= 1;                     
    }
    check_bit >>= 2;                      
  }                               

  return result;                              
}

void readMSGEQ7() {
// Function to read 7 band equalizers
  digitalWrite(res, HIGH);
  digitalWrite(res, LOW);
  for(band=0; band <7; band++) {
    digitalWrite(strobe,LOW); // strobe pin on the shield - kicks the IC up to the next band
    delayMicroseconds(30); //
    left[band] = analogRead(0); // store left band reading
    right[band] = analogRead(1); // ... and the right
    digitalWrite(strobe,HIGH);
  }
}

void shapeMSGEQ7(int _k, boolean initialPass = false) { // Use Welford's algorithm, pass the step 'k' and whether we are on the intiial pass.
  readMSGEQ7();                       // read all 7 bands for left and right channels
  for (band = 0; band < 7; band++) {  // and for each band compute the running average and variance
    tmpAvgL = averageL[band];         // Store old average estimate M_k-1 from previous pass through
    tmpAvgR = averageR[band];  
    averageL[band] = tmpAvgL + ((left[band] - averageL[band])/_k);  // M_k = M_k-1 + (x_k - M_k-1)/k
    averageR[band] = tmpAvgR + ((right[band] - averageR[band])/_k); // Moving '_k' average of left and right channels
    if (!initialPass) {                     // If this is NOT the initial pass, subtract out the zero-point baseline and
      if (_k > 1) {                         // compute the variance if _k > 1 as well
        tmpVarL = varianceL[band];
        tmpVarR = varianceR[band];
        varianceL[band] = ((tmpVarL + ((left[band]-averageL[band])*(left[band]-tmpAvgL)))/(_k-1));
        varianceR[band] = ((tmpVarR + ((right[band]-averageR[band])*(right[band]-tmpAvgR)))/(_k-1));
        stdDevL[band] = findSqRoot(varianceL[band]);
        stdDevR[band] = findSqRoot(varianceR[band]);
      }
      tmpAvgL = averageL[band];              // re-initialize tmpAvgL/R with current baseline values to correct
      tmpAvgR = averageR[band];
      reduce(tmpAvgL, _zeroBSLL[band], 0);
      reduce(tmpAvgR, _zeroBSLR[band], 0);
    }
    left[band] = tmpAvgL;
    right[band] = tmpAvgR;
  }
}   

void setup() {
  Serial.begin(115200);
  pinMode(res, OUTPUT); // reset
  pinMode(strobe, OUTPUT); // strobe
  digitalWrite(res,LOW); // reset low
  digitalWrite(strobe,HIGH); //pin 5 is RESET on the shield
  for (int i = 1; i < (smoothP+1); i++) {
    shapeMSGEQ7(i, _INIT_); // grab band-specific baseline adjustments (assumes no audio on initialization)
  }
  for (band = 0; band < 7; band++) {
    _zeroBSLR[band] = averageR[band];
    _zeroBSLL[band] = averageL[band];
  }
}

void loop() {
  shapeMSGEQ7(smoothP);
   // display values of left channel on serial monitor
  for (band = 0; band < 7; band++) {
    Serial.print(left[band]);
    Serial.print(",");
  }
  // display values of right channel on serial monitor
  for (band = 0; band < 7; band++) {
//    Serial.print(right[band]);
    Serial.print(stdDevL[band]);
    Serial.print(",");
  }
  Serial.println();

  delay(1);
}


