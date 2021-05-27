#include <ADC.h>
#include <Deque.h>
#define  IMPLEMENTATION  FIFO

ADC *adc = new ADC(); // adc object
ADC_Module *adcMod;
const unsigned int micA = A0, micB = A1, micC = A2;
unsigned int analogValA = 0, analogValB = 0, analogValC = 0, counter = 0;
float lastMicros, currMicros, currEventStamp;
float maxTDOA = 135, baseFPS, detectFPS, totalFPS, offsetVal, offsetAngle, finalAngle;;
bool triggered = false;
unsigned int lastEventStamp = 0;
const unsigned int eventThresh = 650;
const int eventLoops = 300, sampleStartIndex = eventLoops;
const int upperBounds_analog = eventLoops * 3;
const int upperBounds_offset = eventLoops * 2;
const int upperBounds_prelog = eventLoops;
const int deque_trigger_number = upperBounds_prelog-1;
unsigned int ABMaxIndex, ACMaxIndex, BCMaxIndex;
int ABTdoa, ACTdoa, BCTdoa;
int analogValsA[upperBounds_analog], analogValsB[upperBounds_analog], analogValsC[upperBounds_analog];
int offsetComparisonAB[upperBounds_offset], offsetComparisonAC[upperBounds_offset], offsetComparisonBC[upperBounds_offset];
Deque<unsigned int> queueA(upperBounds_prelog), queueB(upperBounds_prelog), queueC(upperBounds_prelog);
boolean isCalibrated = false;

//float side_AB;
//float side_AB_TDOA;
//float a_angle;

void setup() {
  adcMod = adc->adc0;
  Serial.begin(20000000);
  pinMode(micA, INPUT);
  pinMode(micB, INPUT);
  pinMode(micC, INPUT);
//  analogReadAveraging(0);
//  analogReadResolution(10);
  adcMod->setAveraging(0); // set number of averages
  adcMod->setResolution(10); // set bits of resolution
  adcMod->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  adcMod->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED ); // ,  change the conversion speed
  adcMod->disableInterrupts();

  Serial.println("Wait for calibration");

}

void loop() {
  counter++;

  analogValA = analogRead(micA);
  analogValB = analogRead(micB);
  analogValC = analogRead(micC);
  if(counter > deque_trigger_number){
    queueA.pop_front();
    queueB.pop_front();
    queueC.pop_front();
  }
  queueA.push_back(analogValA);
  queueB.push_back(analogValB);
  queueC.push_back(analogValC);  
  if((analogValA > eventThresh || analogValB > eventThresh || analogValC > eventThresh) &&  micros()-lastEventStamp > 250000){
    if(isCalibrated){
    currEventStamp = micros();
    for (int i = eventLoops; i < upperBounds_analog; i++){
      analogValsA[i] = analogRead(micA);
      analogValsB[i] = analogRead(micB);
      analogValsC[i] = analogRead(micC);
    }
    totalFPS = micros()-currEventStamp;
    totalFPS = totalFPS/1000000;
    totalFPS = upperBounds_offset/totalFPS;
    totalFPS = ((2*totalFPS)/3)+(baseFPS/3)+2;
//  Serial.println(totalFPS);
    maxTDOA = 0.13 * (totalFPS/340.27);
    triggered = true;
    lastEventStamp = currEventStamp;
    for (int i = 0; i < upperBounds_analog; i++){
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
//      if (i % 2 == 0){
//    
//        Serial.print(-511);
//        Serial.print(",");
//        Serial.print(512);
//        Serial.print(",");
//        Serial.println(analogValsA[i]);
//      }
    }

    
    for(int i = 0; i < upperBounds_offset; i++){
      offsetComparisonAB[i] = 0;
      offsetComparisonAC[i] = 0;
      offsetComparisonBC[i] = 0;
      for(int j = sampleStartIndex; j < sampleStartIndex+eventLoops; j++){
        offsetComparisonAB[i] += analogValsA[j]*analogValsB[j+i-sampleStartIndex];
        offsetComparisonAC[i] += analogValsA[j]*analogValsC[j+i-sampleStartIndex];
        offsetComparisonBC[i] += analogValsB[j]*analogValsC[j+i-sampleStartIndex];
      }
    }
    ABMaxIndex = 0;
    ACMaxIndex = 0;
    BCMaxIndex = 0;
    for(unsigned int i = 0; i < (sizeof(offsetComparisonAB) / sizeof(offsetComparisonAB[0])); i++){
//      Serial.println(offsetComparisonAB[i]);
      if(offsetComparisonAB[i] > offsetComparisonAB[ABMaxIndex]){
        ABMaxIndex = i;
      }
      if(offsetComparisonAC[i] > offsetComparisonAC[ACMaxIndex]){
        ACMaxIndex = i;
      }
      if(offsetComparisonBC[i] > offsetComparisonBC[BCMaxIndex]){
        BCMaxIndex = i;
      }
    }
    ABTdoa = sampleStartIndex-ABMaxIndex; 
    ACTdoa = sampleStartIndex-ACMaxIndex; 
    BCTdoa = sampleStartIndex-BCMaxIndex; 
    
      //What should've been done
//    float side_AB = 0.13;
//    float side_AB_TDOA = (340.27*ABTdoa)/totalFPS;
//    float a_angle = degrees(asin((side_AB_TDOA*sin(90))/0.13));
//    if (isnan(a_angle)){
//        a_angle = 0;
//      }
//    float true_a_angle = 60-a_angle;
//    Serial.println(true_a_angle);
    if(abs(ABTdoa) <= maxTDOA && abs(ACTdoa) <= maxTDOA && abs(BCTdoa) <= maxTDOA && isCalibrated){
      float remapAB = (100.0/maxTDOA)*ABTdoa; //Remapped value to percent. -100 to 100.
      float remapAC = (100.0/maxTDOA)*ACTdoa; 
      float remapBC = (100.0/maxTDOA)*BCTdoa; 

      if(remapAB > 0 && remapBC < 0){
        Serial.print("B is closest || ");
        if (remapAC == 0){
          offsetAngle = 0;
          }
        if (remapAC < 0){
          Serial.print("AB centre towards B");
          offsetAngle = 60.0-map(ABTdoa,0,maxTDOA,0,80);
          }
        if (remapAC > 0){
          Serial.print("BC centre towards B");
         offsetAngle = 300.0+map(BCTdoa,0,-maxTDOA,0,90);   
          }
      }
      else if(remapAC > 0 && remapBC > 0){
        Serial.print("C is closest || ");
        if (remapAB == 0){
          offsetAngle = 240.0;
          }
        if (remapAB < 0){
          Serial.print("AC towards C");
          offsetAngle = 180.0+map(ACTdoa,0,maxTDOA,0,80);
          }
        if (remapAB > 0){
          Serial.print("BC towards C");
          offsetAngle = 300.0-map(BCTdoa,0,maxTDOA,0,80);   
          }
      }
      else if(remapAB < 0 && remapAC < 0){
        Serial.print("A is closest || ");
        if (remapBC == 0){
          offsetAngle = 240.0;
          }
        if (remapBC < 0){
          Serial.print("AB towards A");
          offsetAngle = 60.0+map(ABTdoa,0,-maxTDOA,0,90);
          }
        if (remapBC > 0){
          Serial.print("AC towards A");
          offsetAngle = 120.0+map(BCTdoa,0,maxTDOA,0,90);   
          }
      }
                
        Serial.println("|| angle is "+String(offsetAngle));
        Serial.println("TDOA A->B: " + String(ABTdoa) + " || " + "TDOA A->C: " + String(ACTdoa) + " || " + "TDOA B->C: " + String(BCTdoa));
        Serial.println();
      }
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
