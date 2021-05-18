#include <ADC.h>
#include <limits.h>
#include <Deque.h>
#define  IMPLEMENTATION  FIFO


ADC *adc = new ADC(); // adc object
ADC_Module *adcMod;
const unsigned int micA = A0, micB = A1, micC = A2;
int analogValA = 0, analogValB = 0, analogValC = 0;
unsigned int counter = 0;
float lastMicros, currMicros, detectStamp;
const int activationThreshhold = 700;
boolean triggered = false;
unsigned int lastStamp = 0;
const int activationTimeFrame = 300;
float maxTDOA = 125;
float baseFPS, detectFPS, totalFPS;
float offsetVal, offsetAngle, finalAngle;
const unsigned int angle_face_CB = 300, angle_face_AB = 60, angle_face_AC = 180;
float AMax = 0, BMax = 0 , CMax = 0, AMaxIndex = 0, BMaxIndex = 0, CMaxIndex = 0;
const int upperBounds_analog = activationTimeFrame * 3;
const int upperBounds_offset = activationTimeFrame * 2;
const int upperBounds_prelog = activationTimeFrame;
int analogValsA[upperBounds_analog], analogValsB[upperBounds_analog], analogValsC[upperBounds_analog];
int offsetComparisonAB[upperBounds_offset], offsetComparisonAC[upperBounds_offset], offsetComparisonBC[upperBounds_offset];
Deque<int> queueA(upperBounds_prelog), queueB(upperBounds_prelog), queueC(upperBounds_prelog);
boolean isCalibrated = false;


void setup() {
  adcMod = adc->adc0;
  Serial.begin(2000000);
  pinMode(micA, INPUT);
  pinMode(micB, INPUT);
  pinMode(micC, INPUT);
//  analogReadAveraging(0);
//  analogReadResolution(10);
  adcMod->setAveraging(0); // set number of averages
  adcMod->setResolution(10); // set bits of resolution
  adcMod->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // ,  change the conversion speed
  adcMod->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
  Serial.println("Wait for calibration");

}

void loop() {
  counter++;

  analogValA = analogRead(micA);
  analogValB = analogRead(micB);
  analogValC = analogRead(micC);
  if(counter >= upperBounds_prelog){
    queueA.pop_front();
    queueB.pop_front();
    queueC.pop_front(); 
  }
  queueA.push_back(analogValA);
  queueB.push_back(analogValB);
  queueC.push_back(analogValC);  
  if((analogValA > activationThreshhold || analogValB > activationThreshhold || analogValC > activationThreshhold) &&  micros()-lastStamp > 250000 ){
    detectStamp = micros();
    for (unsigned int i = activationTimeFrame; i < upperBounds_analog; i++){
      analogValsA[i] = analogRead(micA);
      analogValsB[i] = analogRead(micB);
      analogValsC[i] = analogRead(micC);
    }
    totalFPS = micros()-detectStamp;
    totalFPS = totalFPS/1000000;
    totalFPS = upperBounds_offset/totalFPS;
    totalFPS = ((2*totalFPS)/3)+(baseFPS/3)+2;
    maxTDOA = 0.13 * (totalFPS/340.27);
    triggered = true;
    lastStamp = detectStamp;
    AMax = 0, BMax = 0 , CMax = 0, AMaxIndex = 0, BMaxIndex = 0, CMaxIndex = 0, offsetVal = 0, offsetAngle = 0, finalAngle = 0;
    for (unsigned int i = 0; i < upperBounds_analog; i++){
      if(i < upperBounds_prelog){
        analogValsA[i] = map(queueA[i],0,1023,-511, 512);
        analogValsB[i] = map(queueB[i],0,1023,-511, 512);
        analogValsC[i] = map(queueC[i],0,1023,-511, 512);
        }
      else{
        analogValsA[i] = map(analogValsA[i],0,1023,-511, 512);
        analogValsB[i] = map(analogValsB[i],0,1023,-511, 512);
        analogValsC[i] = map(analogValsC[i],0,1023,-511, 512);
        }
    }

    int samplingSize = (sizeof(analogValsA) / sizeof(analogValsA[0]))/3; 
    int samplingStartIndex = samplingSize;
//    int maxSampleIndex = (sizeof(analogValsA) / sizeof(analogValsA[0]))-samplingSize;
    for(unsigned int i = 0; i < upperBounds_offset;i++){
      offsetComparisonAB[i] = 0;
      offsetComparisonAC[i] = 0;
      offsetComparisonBC[i] = 0;
      }
    for(int i = -samplingSize; i < samplingSize; i++){
      for(int j = samplingStartIndex; j < samplingStartIndex+samplingSize; j++){
        offsetComparisonAB[i+samplingSize] += analogValsA[j]*analogValsB[j+i];
        offsetComparisonAC[i+samplingSize] += analogValsA[j]*analogValsC[j+i];
        offsetComparisonBC[i+samplingSize] += analogValsB[j]*analogValsC[j+i];
      }
    }
    unsigned int largestIndexAB = 0;
    unsigned int largestIndexAC = 0;
    unsigned int largestIndexBC = 0;
    for(unsigned int i = 0; i < (sizeof(offsetComparisonAB) / sizeof(offsetComparisonAB[0])); i++){
//      Serial.println(offsetComparisonAB[i]);
      if(offsetComparisonAB[i] > offsetComparisonAB[largestIndexAB]){
        largestIndexAB = i;
      }
      if(offsetComparisonAC[i] > offsetComparisonAC[largestIndexAC]){
        largestIndexAC = i;
      }
      if(offsetComparisonBC[i] > offsetComparisonBC[largestIndexBC]){
        largestIndexBC = i;
      }
    }
    
    int sampleAreaCorrectedAB = samplingStartIndex-largestIndexAB; 
    int sampleAreaCorrectedAC = samplingStartIndex-largestIndexAC; 
    int sampleAreaCorrectedBC = samplingStartIndex-largestIndexBC; 
    if(abs(sampleAreaCorrectedAB) <= maxTDOA && abs(sampleAreaCorrectedAC) <= maxTDOA && abs(sampleAreaCorrectedBC) <= maxTDOA && isCalibrated){
      float remapAB = (100.0/maxTDOA)*sampleAreaCorrectedAB; //Remapped value to percent. -100 to 100.
      float remapAC = (100.0/maxTDOA)*sampleAreaCorrectedAC; 
      float remapBC = (100.0/maxTDOA)*sampleAreaCorrectedBC; 

      if(remapAB > 0 && remapBC < 0){
        Serial.print("B is closest || ");
        if (remapAC == 0){
          offsetAngle = 0;
          }
        if (remapAC < 0){
          Serial.print("AB centre towards B");
          offsetAngle = 60.0-map(sampleAreaCorrectedAB,0,maxTDOA,0,60);
          }
        if (remapAC > 0){
          Serial.print("BC centre towards B");
          offsetAngle = 300.0+map(sampleAreaCorrectedBC,0,-maxTDOA,0,60);   
          }
      }
      else if(remapAC > 0 && remapBC > 0){
        Serial.print("C is closest || ");
        if (remapAB == 0){
          offsetAngle = 240;
          }
        if (remapAB < 0){
          Serial.print("AC towards C");
          offsetAngle = 180.0+map(sampleAreaCorrectedAC,0,maxTDOA,0,60);
          }
        if (remapAB > 0){
          Serial.print("BC towards C");
          offsetAngle = 300.0-map(sampleAreaCorrectedBC,0,maxTDOA,0,60);   
          }
      }
      else if(remapAB < 0 && remapAC < 0){
        Serial.print("A is closest || ");
        if (remapBC == 0){
          offsetAngle = 240;
          }
        if (remapBC < 0){
          Serial.print("AB towards A");
          offsetAngle = 60.0+map(sampleAreaCorrectedAB,0,-maxTDOA,0,60);
          }
        if (remapBC > 0){
          Serial.print("AC towards A");
          offsetAngle = 120+map(sampleAreaCorrectedBC,0,maxTDOA,0,60);   
          }
      }
        
        //Calculate the offset based on diff between a and c.
//        offsetAngle = (remapBC*remapAB);
        Serial.println(" || " + String(offsetAngle));
        Serial.println("TDOA A->B: " + String(sampleAreaCorrectedAB) + " || " + "TDOA A->C: " + String(sampleAreaCorrectedAC) + " || " + "TDOA B->C: " + String(sampleAreaCorrectedBC));
        Serial.println();
      }
    else if(isCalibrated){
      
//      Serial.println("A signal had best correllation outside the theorhetical signal distance");
    }
  }
    else if(counter > 1000000){
      if(!triggered){
        currMicros = micros();
        currMicros = currMicros/1000000;
        baseFPS = counter/(currMicros-lastMicros);
        if(!isCalibrated){
          Serial.println("System is now calibrated");
          isCalibrated = true;
        }
        lastMicros = currMicros;
        counter = 0;
        }
      else{
        currMicros = micros();
        currMicros = currMicros/1000000;
        lastMicros = currMicros;
        counter = 0;
        triggered = false;
        }
    }


}
