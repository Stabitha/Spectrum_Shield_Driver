// Current Issues: Detection of changepoints implemented, but they still need tweaking. They should still be useful for detecting when instruments enter and exit a 
//    composition with the right threshold value (probably depends on specific audio connections, unfortunately).

// MSGEQ7 Control
int strobe = 4; // strobe pins on digital 4
int res = 5; // reset pins on digital 5
static const byte smoothP = 64;  // Number of samples to compute rolling average over (empirically set)
static const boolean _INIT_ = true;        // increasing 'smoothP' makes bars jutter less, but also increases <S-<S>> = c 
static const byte tmpSqRt = 5;            // sqroot(smoothP/2) = sqroot(32) = 5.65 = 5;

long checkL[7] = { 0, 0, 0, 0, 0, 0, 0};
long checkR[7] = { 0, 0, 0, 0, 0, 0, 0};
                    
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
boolean chgPtL[7] = { false, false, false, false, false, false, false};
boolean chgPtR[7] = { false, false, false, false, false, false, false};
long cumSumSqL[7] = { 0, 0, 0, 0, 0, 0, 0};
long cumSumSqR[7] = { 0, 0, 0, 0, 0, 0, 0};
long tmpD_kL = 0;
long tmpD_kR = 0;
long D_kL[7] = { 0, 0, 0, 0, 0, 0, 0};
long D_kR[7] = { 0, 0, 0, 0, 0, 0, 0};
uint32_t initValueL = 0;
uint32_t initValueR = 0;
uint32_t tmpVarL = 0;
uint32_t tmpVarR = 0;
int prvL[7] = { 0, 0, 0, 0, 0, 0, 0};
int prvR[7] = { 0, 0, 0, 0, 0, 0, 0};
int outputL[7] = { 0, 0, 0, 0, 0, 0, 0};
int outputR[7] = { 0, 0, 0, 0, 0, 0, 0};
int tmpAvgL = 0;
int tmpAvgR = 0;
int band;                                        // counting variable for going through channels

inline void reduce(int &anInt, int aAmount, int aLimit, int aMin = 0) {
  int r = ((aAmount > aLimit) ? (anInt-aLimit) : (anInt-aAmount));
  if (r < aMin)
    anInt = aMin;
  else
    anInt = r;
}

inline void increase(int &anInt, int aAmount, int aLimit, int aMax = 1023) {
  int r = ((aAmount > aLimit) ? (anInt+aLimit) : (anInt+aAmount));
  if (r > aMax)
    anInt = aMax;
  else
    anInt = r;
}

inline byte increaseByte(byte aByte, byte aAmount, byte aLimit, byte aMax) {
  int r = ((aAmount > aLimit) ? (aByte+aLimit) : (aByte+aAmount));
  if (r > aMax)
    return aMax;
  else
    return r;
}

inline byte reduceByte(byte aByte, byte aAmount, byte aLimit, byte aMin) {
  int r = ((aAmount > aLimit) ? (aByte-aLimit) : (aByte-aAmount));
  if (r < aMin)
    return aMin;
  else
    return r;
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

void resetCS(int aBand) {
    D_kL[aBand] = 0;
    D_kR[aBand] = 0;
    cumSumSqL[aBand] = 0;
    cumSumSqR[aBand] = 0;
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
    if (!initialPass) {
      reduce(left[band], _zeroBSLL[band], 1023, 0);
      reduce(right[band], _zeroBSLR[band], 1023, 0);
    }  
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
        cumSumSqL[band] += ((prvL[band]-tmpAvgL)*(prvL[band]-tmpAvgL));       // NOTE: cumSumSq is reset to zero in resetCS();
        cumSumSqR[band] += ((prvR[band]-tmpAvgR)*(prvR[band]-tmpAvgR));       
        prvL[band] = left[band];
        prvR[band] = right[band];      
        tmpD_kL = D_kL[band];
        tmpD_kR = D_kR[band];
        D_kL[band] = (long)((_k*cumSumSqL[band]*tmpD_kL + ((left[band]-averageL[band])*(left[band]-averageL[band])))/(_k*cumSumSqL[band]));
        D_kR[band] = (long)((_k*cumSumSqR[band]*tmpD_kR + ((right[band]-averageR[band])*(right[band]-averageR[band])))/(_k*cumSumSqR[band]));
        tmpD_kL = (tmpSqRt*D_kL[band]);
        tmpD_kR = (tmpSqRt*D_kR[band]);
        tmpD_kL = (D_kL[band] > 0) ? tmpD_kL : -1*(tmpD_kL);
        tmpD_kR = (D_kR[band] > 0) ? tmpD_kR : -1*(tmpD_kR);
        if (tmpD_kL > 163) { // signal chgpt detected, using heuristic value, slightly larger than Inclan & Tiao
//       resetCS(band);
          chgPtL[band] = true;
        } else {
          chgPtL[band] = false;
        }
        if (tmpD_kR > 163) {
//       resetCS(band);
          chgPtR[band] = true;
        } else {
          chgPtR[band] = false;
        }
      checkL[band] = tmpD_kL;
      checkR[band] = tmpD_kR;
      }
      tmpAvgL = averageL[band];              // re-initialize tmpAvgL/R with current baseline values to correct
      tmpAvgR = averageR[band];
 //   reduce(tmpAvgL, _zeroBSLL[band], 0);
 //   reduce(tmpAvgR, _zeroBSLR[band], 0);    
      left[band] = tmpAvgL;
      right[band] = tmpAvgR;
    }
  }
}   

inline void limitLeft(int aLimit) {
  for (band = 0; band < 7; band++) {
    byte newS = 0;
    byte oldS = 0;
    byte diff = 0;
    newS = outputR[band];
    oldS = prvL[band];
    diff = ((newS < oldS) ? (oldS - newS) : (newS - oldS));
    prvL[band] = ((newS < oldS) ? (reduceByte(oldS, diff, aLimit, 0)) : (increaseByte(oldS, diff, aLimit, 255)));
  }
}

inline void limitRight(int aLimit) {
  for (band = 0; band < 7; band++) {
    byte newS = 0;
    byte oldS = 0;
    byte diff = 0;
    newS = outputR[band];
    oldS = prvR[band];
    diff = ((newS < oldS) ? (oldS - newS) : (newS - oldS));
    prvR[band] = ((newS < oldS) ? (reduceByte(oldS, diff, aLimit, 0)) : (increaseByte(oldS, diff, aLimit, 255)));
  }
}
    
byte cnt = 1;

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
  li
  if (cnt > 64) {                          // <----- change this value for different sample sizes
    for (band = 0; band < 7; band++) {
      resetCS(band);
    }
  }
  cnt++;
   // display values of left channel on serial monitor
  for (band = 0; band < 7; band++) {
    Serial.print(left[band]);
//    Serial.print(checkL[band]);
//    Serial.print(chgPtL[band]);
    Serial.print(",");
  }
  // display values of right channel on serial monitor (or uncomment lines to debug)
  for (band = 0; band < 7; band++) {
    Serial.print(right[band]);
//    Serial.print(stdDevL[band]);       // check standard deviation output on left channel
//    Serial.print(D_kL[band]);          // check D_k statistic for the left channel
//    Serial.print(checkR[band]);        // D_K*tmpSqrRt/100;
//    Serial.print(chgPtR[band]);         // check whether changepoints are detected
    Serial.print(",");
  }
  Serial.println();

  delay(1);
}


