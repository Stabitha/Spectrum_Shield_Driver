// Current Issues: Sometimes when the input level is changed a large amount the value in bslL/bslR is >graphTop
//                 After driving non-zero input the values sent by Serial.print() do not return to zero, although 
//                   stopping Processing sketch or Arduino Serial Monitor and restarting them restores zero values.
//                 Does not compute running variance/standard deviation which could be useful for indicating 
//                   changes in the input

// Sparkfun Spectrum Shield Driver v0.1 (contributors: Tom Flock, John Boxall, Elijah Gregory)
// Written using Arduino IDE v1.0.5 and Processing v2.0.3 on Nov 3rd, 2013. by Tom Flock
// Based on Example 48.1 - tronixstuff.com/tutorials > chapter 48 - 30 Jan 2013 by John Boxall
// Modified June 25th, 2014 by Elijah J. Gregory

// Currently the driver computes the running average upon initialization and uses this value
// to baseline correct later values. A 'smoothP' point running average is performed so that
// printed values are smoothed out in time.

// This code receives multiplexed data from the Spectrum Shield's two MSGEQ7 ICs
// and transmits the values via serial at a 115200 baud rate.
// The fourteen values are seperated by commas and are terminated by a newline,
// which is the format your visualization code should expect.

// MSGEQ7 Control
int strobe = 4; // strobe pins on digital 4
int res = 5; // reset pins on digital 5
static const byte smoothP = 64;  // Number of samples to compute rolling average over (empirically set)
static const boolean _INIT_ = true;        // increasing 'smoothP' makes bars jutter less, but also increases <S-<S>> = c 
                                 
int left[7];                                     // store raw band values in these arrays
int right[7];
int bslR[7] = { 0, 0, 0, 0, 0, 0, 0};            // zero-point baseline corrected bands
int bslL[7] = { 0, 0, 0, 0, 0, 0, 0};
int baselineRight[7] = { 0, 0, 0, 0, 0, 0, 0};   // 'smoothP' running average of raw data 
int baselineLeft[7] = { 0, 0, 0, 0, 0, 0, 0};
int _zeroBSLR[7] = { 0, 0, 0, 0, 0, 0, 0};       // bands with zero volume input
int _zeroBSLL[7] = { 0, 0, 0, 0, 0, 0, 0};
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

void shapeMSGEQ7(int stopPos, boolean initialPass = false) { // Use Welford's algorithm
  readMSGEQ7();                       // read all 7 bands for left and right channels
  for (band = 0; band < 7; band++) {  // and for each band compute the running average and variance
    baselineLeft[band] += ((left[band] - baselineLeft[band])/stopPos);  // M_k = M_k-1 + (x_k - M_k-1)/k
    baselineRight[band] += ((right[band] - baselineRight[band])/stopPos); // Moving 'stopPos' average of left and right channels
    bslL[band] = baselineLeft[band];  // stores M_k
    bslR[band] = baselineRight[band];
    if (!initialPass) {    
      reduce(bslL[band], _zeroBSLL[band], 0);
      reduce(bslR[band], _zeroBSLR[band], 0);
    }
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
    _zeroBSLR[band] = baselineRight[band];
    _zeroBSLL[band] = baselineLeft[band];
  }
}

void loop() {
  shapeMSGEQ7(smoothP);
   // display values of left channel on serial monitor
  for (band = 0; band < 7; band++) {
    Serial.print(bslL[band]);
    Serial.print(",");
  }
  // display values of right channel on serial monitor
  for (band = 0; band < 7; band++) {
    Serial.print(bslR[band]);
    Serial.print(",");
  }
  Serial.println();

  delay(1);
}


