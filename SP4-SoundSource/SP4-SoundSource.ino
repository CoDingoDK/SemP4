#include <ADC.h>
#include <limits.h>
#include <Deque.h>
#include <ArduinoQueue.h>
#define  IMPLEMENTATION  FIFO


ADC *adc = new ADC(); // adc object
const unsigned int micA = A0, micB = A1, micC = A2;
int analogValA = 0, analogValB = 0, analogValC = 0;
unsigned int counter = 0;
unsigned long lastMillis;
unsigned long currMillis;
unsigned long fCount;
const int activationThreshhold = 700;
unsigned int lastStamp = 0;
const int activationTimeFrame = 600;
unsigned int framesPerSecond;
float offsetVal, offsetAngle, finalAngle;
const unsigned int angleA = 300, angleB = 60, angleC = 180;
float AMax = 0, BMax = 0 , CMax = 0, AMaxIndex = 0, BMaxIndex = 0, CMaxIndex = 0;
int analogValsA[activationTimeFrame*3], analogValsB[activationTimeFrame*3], analogValsC[activationTimeFrame*3];
int offsetComparisonAB[activationTimeFrame*2], offsetComparisonAC[activationTimeFrame*2], offsetComparisonBC[activationTimeFrame*2];
Deque<int> queueA(activationTimeFrame), queueB(activationTimeFrame), queueC(activationTimeFrame);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(2000000);
  pinMode(micA, INPUT);
  pinMode(micB, INPUT);
  pinMode(micC, INPUT);
//  analogReadAveraging(0);
//  analogReadResolution(10);
  adc->adc0->setAveraging(0); // set number of averages
  adc->adc0->setResolution(10); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // ,  change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
}

void loop() {
  if(counter > 1000000){
    currMillis = millis();
    framesPerSecond = (counter/(currMillis-lastMillis)*1000);
//    Serial.println(framesPerSecond);
    fCount = 0;
    lastMillis = currMillis;
    counter = 0;
    }
  analogValA = adc->adc0->analogRead(micA);
  analogValB = adc->adc0->analogRead(micB);
  analogValC = adc->adc0->analogRead(micC);
  counter++;
  if(counter >= activationTimeFrame){
    queueA.pop_front();
    queueB.pop_front();
    queueC.pop_front(); 
  }
  queueA.push_back(analogValA);
  queueB.push_back(analogValB);
  queueC.push_back(analogValC);  
  if((analogValA >= activationThreshhold or analogValB >= activationThreshhold or analogValC >= activationThreshhold) and millis()-lastStamp >= 500){
    
//    Serial.println("Signal discovered, recording " + String(activationTimeFrame) +  " frames");
    for (unsigned int i = activationTimeFrame; i < (sizeof(analogValsA) / sizeof(analogValsA[0])); i++){
      analogValsA[i] = adc->adc0->analogRead(micA);
      analogValsB[i] = adc->adc0->analogRead(micB);
      analogValsC[i] = adc->adc0->analogRead(micC);
    }
    lastStamp = millis();
    AMax = 0, BMax = 0 , CMax = 0, AMaxIndex = 0, BMaxIndex = 0, CMaxIndex = 0, offsetVal = 0, offsetAngle = 0, finalAngle = 0;
    for (unsigned int i = 0; i < (sizeof(analogValsA) / sizeof(analogValsA[0])); i++){
      
      if(i < activationTimeFrame){
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

    counter += (sizeof(analogValsA) / sizeof(analogValsA[0]));
    for (unsigned int i = 0; i < (sizeof(analogValsA) / sizeof(analogValsA[0])); i++){
      if (analogValsA[i] > AMax){
        AMax = analogValsA[i];
        AMaxIndex = i;
      }
      if (analogValsB[i] > BMax){
        BMax = analogValsB[i];
        BMaxIndex = i;
      }
      if (analogValsC[i] > CMax){
        CMax = analogValsC[i];
        CMaxIndex = i;
      }
    }

    int samplingSize = (sizeof(analogValsA) / sizeof(analogValsA[0]))/3; 
    int samplingStartIndex = samplingSize;
    int maxSampleIndex = (sizeof(analogValsA) / sizeof(analogValsA[0]))-samplingSize;
    for(unsigned int i = 0; i < (sizeof(offsetComparisonAB) / sizeof(offsetComparisonAB[0]));i++){
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
    Serial.println("Travel time A->B: " + String(sampleAreaCorrectedAB) + " || " + "Travel time A->C: " + String(sampleAreaCorrectedAC) + " || " + "Travel time B->C: " + String(sampleAreaCorrectedBC));
    if(sampleAreaCorrectedAB < 0){
      }
    Serial.println("Amplitude A: ");
    if(AMaxIndex <= BMaxIndex and AMaxIndex <= CMaxIndex){
      unsigned int smallest = min(BMaxIndex, CMaxIndex);
      unsigned int largest = max(BMaxIndex, CMaxIndex);
      
      offsetVal = (smallest-AMaxIndex)/(largest-AMaxIndex);
      
      offsetAngle = map(offsetVal, 0, 1, 0, 120);
      Serial.println("A is closest, B and C have offset val: " + String(offsetVal));
      }
//    Serial.println("Done recording, offset angle found at " + String(offsetAngle) + " degrees");
//    Serial.println();
    }


}
