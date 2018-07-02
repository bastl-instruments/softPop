#include <EEPROM.h>


/*

 
 Writen by Vaclav Pelousek 2017
 open source license: CC BY SA
 http://www.bastl-instruments.com
 
-code running in the softPop
  -pattern generator & quantizer
 
 -software written in Arduino 1.0.6 - used to flash ATTINY 85 running at 8mHz
 -created with help of the heavenly powers of internet and several tutorials that you can google out
 -i hope somebody finds this code usefull
 
 thanks to 
 -Lennart Schierling for making some work on the register access
 -Uwe Schuller for explaining the capacitance of zener diodes
 -Peter Edwards for making the inspireing bitRanger
 -Rob Hordijk for inventing the amazing concept of the Rungler
 -Ondrej Merta for being the best boss
 -and the whole bastl crew that made this project possible
 
 */

#define F_CPU 8000000  // This is used by delay.h library
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>        // Adds useful constants
#include <util/delay.h>    // Adds delay_ms and delay_us functions


//debugging purposes
//#include <SoftwareSerial.h>
//#define rxPin 5    
//#define txPin 3
//SoftwareSerial serial(rxPin, txPin);

uint8_t analogChannelRead=1;
uint16_t analogValues[3];
uint16_t lastAnalogValues[3];
uint8_t runglerByte;
int pwmCounter;
uint16_t upIncrement=0;
uint16_t downIncrement=255;
uint32_t _upIncrement=0;
uint32_t _downIncrement=255;
uint8_t pwmIncrement;
uint8_t waveshape,lastWaveshape;
long _value;
bool goingUp;
uint16_t counter;
bool ptrnB=false, ptrnA=true;
bool resetState=false;
uint16_t runglerOut;
bool lastDoReset;
const uint8_t runglerMap[8]={
  0,80,120,150,180,200,220,255};

//uint16_t wsMap[10]={ 0,120,150,180,255,   20,60,120,190,254};


uint8_t analogPins[3]={
  A1,A2,A3};

uint8_t _xor;
int _val;
bool _gate;
int out,in;
bool render;
bool cycle;
const bool usePin[4]={
  true,false,true,false};
uint8_t lfoValue=0;
bool lfoFlop=true;
bool doReset=false;
bool firstRead=false;
const uint8_t analogToDigitalPinMapping[4]={
  7,PORTB2,PORTB4,PORTB3};


uint16_t wsMap[10]={ 
  0,60,127,191,255,   50,127,190,230,254};

#define WSMAP_POINTS 5

//uint8_t mapLookup[256];
/*
void createLookup(){
 for(uint16_t i=0;i<256;i++){
 mapLookup[i]=curveMap(i,WSMAP_POINTS,wsMap);
 }
 }
 */

uint32_t curveMap(uint8_t value, uint8_t numberOfPoints, uint16_t * tableMap){
  uint32_t inMin=0, inMax=255, outMin=0, outMax=255;
  for(int i=0;i<numberOfPoints-1;i++){
    if(value >= tableMap[i] && value <= tableMap[i+1]) {
      inMax=tableMap[i+1];
      inMin=tableMap[i];
      outMax=tableMap[numberOfPoints+i+1];
      outMin=tableMap[numberOfPoints+i];
      i=numberOfPoints+10;
    }
  }
  return map(value,inMin,inMax,outMin,outMax);
}

void configureExternalPinChangeInterrupt(){

  // bitWrite(MCUCR,ISC01,1);
  // bitWrite(MCUCR,ISC00,1);
  //interrupt only on the rising edge
  PCMSK=0;
  bitWrite(PCMSK,PCINT3,1); //pinchange mask allows only one pin to trigger interrupt

  //bitWrite(GIMSK,INT0,1); //maybe needed probably not
  bitWrite(GIMSK,PCIE,1);
  // interrupt enable
}
uint32_t _clocks;
uint32_t clocks(){
  //return _clocks;
  uint32_t _t=_clocks<<6;
  return (TCNT0>>2)|(_t);//_clocks;
}

ISR(TIMER0_OVF_vect){ // increment _clocks at PWM interrupt overflow - this gives 16bit time consciousnes to the chip (aproxx 2 seconds before overflow)
  // bitWrite(PORTB,2,1);
  _clocks++; 
  //  bitWrite(PORTB,2,0);
}

uint32_t lastTime;
uint32_t duration=0, lastDuration=0, subDuration=0;
//uint16_t periodTable[61]={
// 1136,1073,1012,956,902,851,804,758,716,676,638,602,568,536,506,478,451,426,402,379,358,338,319,301,284,268,253,239,225,213,201,190,179,169,159,150,142,134,127,119,113,106,100,95,89,84,80,75,71,67,63,60,56,53,50,47,45,42,40,38,36};
#define NUMBER_OF_NOTES 73
/*
uint16_t periodTable[NUMBER_OF_NOTES]={ //31250
 36364,34323,32396,30578,28862,27242,25713,24270,22907,21622,20409,19263,18182,17161,16198,15289,14431,13621,12856,12135,11454,10811,10204,9631,9091,8581,8099,
 7645,7215,6811,6428,6067,5727,5405,5102,4816,4545,4290,4050,3822,3608,3405,3214,3034,2863,2703,2551,2408,2273,2145,2025,1911,1804,1703,1607,1517,1432,1351,1276,1204,1136,1073,1012,956,902,851,804,758,716,676,638,602,568};
 */
//32400
uint16_t periodTable[NUMBER_OF_NOTES]={
  37673,35559,33562,31679,29901,28223,26639,25144,23732,22400,21143,19956,18836,17779,16781,15840,14950,14111,13319,12572,11866,11200,10572,9978,9418,8890,8391,7920,7475,7056,6660,6286,5933,5600,5286,4989,4709,4445,4195,3960,3738,3528,3330,3143,2967,2800,2643,2495,2355,2222,2098,1980,1869,1764,1665,1571,1483,1400,1321,1247,1177,1111,1049,990,934,882,832,786,742,700,661,624,589};


uint8_t note;
//A0up to A5 = 5 octaves
//31250 hz increments
//440hz ? increments 31250/440 = 71 increments 

bool above;
uint8_t coarseTuneValue;
uint8_t fineTuneValue;
bool flop=false;
uint8_t PWM_STEPS_PER_SEMITONE=100;
uint8_t countDown;


//chroma
//maj dia
//maj penta
//maj blues C, D, D♯/E♭, E, G, A
//{1,0,1,0,1,0,1,0,1,0,1,0}, //whole tone //maj chord //maj fifth //maj +7  //min dia //min penta  //min blues  //min chord //whole tone scale //min +7


//bool quantizeTable[12];

bool scales[13][12]={
  {
    1,1,1,1,1,1,1,1,1,1,1,1                        }
  , //chroma

  {
    1,0,1,0,1,1,0,1,0,1,0,1                        }
  , //maj dia
  {
    1,0,1,0,1,0,0,1,0,1,0,0                        }
  , //maj penta
  {
    1,0,1,1,1,0,0,1,0,1,0,0                        }
  , //maj blues C, D, D♯/E♭, E, G, A
  //{1,0,1,0,1,0,1,0,1,0,1,0}, //whole tone
  {
    1,0,0,0,1,0,0,1,0,0,0,0                        }
  , //maj chord
  {
    1,0,0,0,0,0,0,1,0,0,0,0                        }
  , //maj fifth
  {
    1,0,0,0,1,0,0,1,0,0,1,0                        }
  , //maj +7

  {
    1,0,1,1,0,1,0,1,1,0,1,0                        }
  , //min dia
  {
    1,0,0,1,0,1,0,1,0,0,1,0                        }
  , //min penta
  {
    1,0,0,1,0,1,1,1,0,0,1,0                        }
  , //min blues
  {
    1,0,0,1,0,0,0,1,0,0,0,0                        }
  , //min chord
  {
    1,0,1,0,1,0,1,0,1,0,1,0                        }
  , //whole tone scale
  {
    1,0,0,1,0,0,0,1,0,0,1,0                        }
  , //min +7
};
uint8_t quantizeNote(uint8_t _scale,uint8_t note){
  if(scales[_scale][note%12]){
    return note;
  }
  else{
    uint8_t higher=0;
    while(!scales[_scale][(note+higher)%12]){
      higher++;
    }
    uint8_t lower=0;
    while(!scales[_scale][(note-lower)%12]){
      lower++;
    }
    if(higher<lower) return note+higher;
    else return note-lower;
  }
  return note;
}

bool autoTune=true;
bool rec=false;
bool play=false;
uint8_t sequenceCounter;
uint8_t sequenceLength[2];
#define MAX_SEQUENCE_LENGTH 64
uint8_t sequence[2][MAX_SEQUENCE_LENGTH];

void clear(){
  rec=false;
  play=false; 
  sequenceCounter=0;

}
bool ers;
void erase(){
  ers=true;
}

void record(){

  rec=true;
  play=true; 
  sequenceCounter=0;
  sequence[ptrnB][sequenceCounter]=note;
  sequenceLength[ptrnB]=0;

}
void function(){

}



ISR(PCINT0_vect)  //pinchange interrupt
{
  // flop=!flop;
  /// digitalWrite(2,flop);

  if(bitRead(PINB,PINB3)){ //detect rising edge


    //  duration+=newDuration; //make average value from the last two durations
    // duration=duration>>1;
    // duration=newDuration;
    // if(abs(lastDuration-duration)<10){ //is the frequency somehow stable - should I analyse in the first place?
    if(countDown>0) countDown--;
    if(countDown==0){

      uint32_t newDuration=clocks()-lastTime; //measure duration
      lastTime=clocks();


      lastDuration=duration; // remember last duration
      // newDuration+=lastDuration;
      duration=newDuration;//>>1;
      subDuration=duration/16;
      countDown=15;
      for(int i=NUMBER_OF_NOTES-1;i>0;i--){ //first find out which note is the closest and whether the frequency is above or bellow
        if(subDuration<=periodTable[i]){
          if(subDuration>(periodTable[i]+((periodTable[i]-periodTable[i+1])/2))) note=i, above=true;
          else note=i+1, above=false;
          i=0;
        }
      }
      //note=quantizeNote(2,note);
      // coarseTuneValue=note<<2;
      // if(abs(duration-periodTable[note])<10) fineTuneValue=255;//note<<2;
      // else fineTuneValue=0;

      // if(!(note%12)) bitWrite(PORTB,2,1);//coarseTuneValue=255;
      //  else bitWrite(PORTB,2,0);//coarseTuneValue=0;

      uint16_t absoluteTuningError=abs(subDuration-periodTable[note]);
      uint16_t relativeTuningError=constrain(map(absoluteTuningError,0,periodTable[note]-periodTable[note+1],0,PWM_STEPS_PER_SEMITONE),0,PWM_STEPS_PER_SEMITONE);//255);//PWM_STEPS_PER_SEMITONE);
      if(autoTune){
        if(relativeTuningError>5){

          // bitWrite(PORTB,2,1);
          uint8_t pwmChange=0;
          if(above){ 
            pwmChange=relativeTuningError;//constrain(map(absoluteTuningError,periodTable[note+1],periodTable[note],0,PWM_STEPS_PER_SEMITONE),0,PWM_STEPS_PER_SEMITONE);
            if((fineTuneValue-pwmChange)<0) fineTuneValue+=PWM_STEPS_PER_SEMITONE-pwmChange, note++;
            else fineTuneValue-=pwmChange;
          }
          else {
            pwmChange=relativeTuningError;//constrain(map(absoluteTuningError,periodTable[note],periodTable[note-1],0,PWM_STEPS_PER_SEMITONE),0,PWM_STEPS_PER_SEMITONE);
            if((fineTuneValue+pwmChange)>255) fineTuneValue-=PWM_STEPS_PER_SEMITONE-pwmChange, note--;
            else fineTuneValue+=pwmChange;
          }
          //bitWrite(PORTB,2,0);

        }
      }
      else{
        fineTuneValue=0;
      }
      // uint8_t relativeTuningError=((periodTable[note]-periodTable[note+1]) * abs(periodTable[note]-absoluteTuningError))>>4;


      // digitalWrite(2,note%12);


      OCR0B= coarseTuneValue;
      OCR0A= fineTuneValue;


    }
  }
}



void setup()  { 
  load();
  digitalWrite(5,HIGH);
  pinMode(4, INPUT);
  digitalWrite(4,HIGH);
  //createLookup();
  setTimers(); //setup audiorate interrupt
  runglerByte=random(255);
  pinMode(0, OUTPUT);
  /*
  pinMode(A1, INPUT);
   pinMode(A2, INPUT);
   pinMode(A3, INPUT);
   */
  pinMode(1, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  digitalWrite(4,LOW);
  init();
  connectChannel(0);
  startConversion();
  setFrequency(300);
  /*
  for(uint8_t i=0;i<64;i++){
   sequence[0][i]==127;
   sequence[1][i]==127;
   }
   */
} 

void setTimers(void)
{
  /*
  TCCR0A=0;
   TCCR0B=0;
   bitWrite(TCCR0A,COM0A0,0);
   bitWrite(TCCR0A,COM0A1,1);
   bitWrite(TCCR0A,COM0B0,0);
   bitWrite(TCCR0A,COM0B1,1);
   bitWrite(TCCR0A,WGM00,1);
   bitWrite(TCCR0A,WGM01,1);
   bitWrite(TCCR0B,WGM02,0);
   bitWrite(TCCR0B,CS00,1);
   */
  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;
  TCCR0B = 0<<WGM02 | 1<<CS00;

  //  setup timer 0 to run fast for audiorate interrupt 
  TCCR1 = 0;                  //stop the timer
  TCNT1 = 0;                  //zero the timer
  GTCCR = _BV(PSR1);          //reset the prescaler
  OCR1A = 250;                //set the compare value
  // OCR1C = 31;
  //TIMSK = _BV(OCIE1A);        //interrupt on Compare Match A
  //start timer, ctc mode, prescaler clk/1    
  //TCCR1 = _BV(CTC1) | _BV(CS12);//  | _BV(CS11) ;//| _BV(CS10); //| _BV(CS13) | _BV(CS12) | _BV(CS11) |
  //bitWrite(TCCR1,CS12,0);

  sei();
  TIMSK |=_BV(TOIE0);   //enable overflow interrupt - used for counting time
  configureExternalPinChangeInterrupt();
}

uint16_t buttonAnalogValue, lastButtonAnalogValue, preLastButtonAnalogValue;
#define NO_PRESS_THRES 980
#define THR_1 962
#define THR_2 909
#define THR_3 880
#define THR_4 780
#define NUMBER_OF_USEFULL_LENGTHS 7
uint8_t usefullLengths[NUMBER_OF_USEFULL_LENGTHS]={
  2,4,8,16,32,48,64};

void patternB(){
  if(play && ptrnB) play=false;
  else if(play && ptrnA) play=true,ptrnA=false,ptrnB=true;
  else if(!play) play=true, ptrnA=false, ptrnB=true, sequenceCounter=0;

}
void patternA(){
  if(play && ptrnA) play=false;
  else if(play && ptrnB) play=true,ptrnA=true,ptrnB=false;
  else if(!play) play=true, ptrnA=true, ptrnB=false, sequenceCounter=0;
}
void load(){
  sequenceLength[1]=EEPROM.read((2*MAX_SEQUENCE_LENGTH)+1);
  sequenceLength[0]=EEPROM.read((2*MAX_SEQUENCE_LENGTH));
  autoTune= EEPROM.read((2*MAX_SEQUENCE_LENGTH)+2);
  ptrnB= EEPROM.read((2*MAX_SEQUENCE_LENGTH)+3);
  play= EEPROM.read((2*MAX_SEQUENCE_LENGTH)+4);
  ptrnA= EEPROM.read((2*MAX_SEQUENCE_LENGTH)+5);

  for(uint8_t i=0; i<MAX_SEQUENCE_LENGTH;i++){
    sequence[0][i] =EEPROM.read(i);
    sequence[1][i] =EEPROM.read(i+MAX_SEQUENCE_LENGTH);
  }
}
void save(uint8_t _ptrn, uint8_t _length){
  if(_ptrn){
    EEPROM.write((2*MAX_SEQUENCE_LENGTH)+1,_length);
    for(uint8_t i=0; i<_length;i++){
      EEPROM.write(i+MAX_SEQUENCE_LENGTH,sequence[_ptrn][i]);
    }
  }
  else{
    EEPROM.write((2*MAX_SEQUENCE_LENGTH),_length);
    for(uint8_t i=0; i<_length;i++){
      EEPROM.write(i,sequence[_ptrn][i]);
    }
  }
}
void saveState(){
  EEPROM.write((2*MAX_SEQUENCE_LENGTH)+2,autoTune);
  EEPROM.write((2*MAX_SEQUENCE_LENGTH)+3,ptrnB);
  EEPROM.write((2*MAX_SEQUENCE_LENGTH)+4,play);
  EEPROM.write((2*MAX_SEQUENCE_LENGTH)+5,ptrnA);
}
ISR(ADC_vect){
  /*
  if(!firstRead){
   lastAnalogValues[analogChannelRead]=analogValues[analogChannelRead];
   analogValues[analogChannelRead]= getConversionResult();
   if(analogChannelRead==2 &&  lastAnalogValues[2]!= analogValues[2]) setFrequency(mapLookup[analogValues[2]>>2]);
   
   analogChannelRead++;
   while(!usePin[analogChannelRead]){
   analogChannelRead++;
   if(analogChannelRead>3) analogChannelRead=0;
   }
   
   if(analogChannelRead>2) analogChannelRead=2;
   connectChannel(analogChannelRead);
   firstRead=true;
   startConversion();
   }
   else {
   if(analogChannelRead==2){
   if(analogValues[analogChannelRead]<750) bitWrite(DDRB,analogToDigitalPinMapping[analogChannelRead],1);
   bitWrite(DDRB,analogToDigitalPinMapping[analogChannelRead],0);
   bitWrite(PORTB,analogToDigitalPinMapping[analogChannelRead],0);
   }
   firstRead=false;
   startConversion();
   }
   */
  uint16_t newValue=getConversionResult();

  preLastButtonAnalogValue=lastButtonAnalogValue;
  lastButtonAnalogValue=buttonAnalogValue;
  buttonAnalogValue=newValue;
  // if(abs(newValue-buttonAnalogValue)<10)  //debounce


    if(preLastButtonAnalogValue>=NO_PRESS_THRES && buttonAnalogValue<NO_PRESS_THRES && lastButtonAnalogValue<NO_PRESS_THRES){ //buttonPress
    if(buttonAnalogValue<THR_4) patternA(), saveState();  // B coarseTuneValue=255;//ERASE 
    else if(buttonAnalogValue<THR_3) patternB(), saveState(); //recordcoarseTuneValue=192;//\rec 
    else if(buttonAnalogValue<THR_2) autoTune=!autoTune, saveState();// coarseTuneValue=128;// QUANTIZE 
    else if(buttonAnalogValue<THR_1) record(); //A coarseTuneValue=64;// CLEAR 
  }
  if(buttonAnalogValue>NO_PRESS_THRES){
    //coarseTuneValue=0;
  }
  if(lastButtonAnalogValue<NO_PRESS_THRES && buttonAnalogValue>=NO_PRESS_THRES){ //buttonRelease
    if(rec){

      rec=false;

      play=true; 
      if(sequenceLength[ptrnB]==0){
        sequenceLength[ptrnB]=sequenceCounter;
        if(sequenceLength[ptrnB]==3 || sequenceLength[ptrnB]==12 || sequenceLength[ptrnB]==24){ // add exceptions
        }
        else{
          for(uint8_t i=0;i<NUMBER_OF_USEFULL_LENGTHS;i++){
            if(usefullLengths[i]>sequenceCounter) {
              if(abs(usefullLengths[i]-sequenceCounter)>abs(usefullLengths[i-1]-sequenceCounter)) sequenceLength[ptrnB]=usefullLengths[i-1], i=100;
              else sequenceLength[ptrnB]=usefullLengths[i], i=100;
            }

          }
        }
      }
      save(ptrnB,sequenceLength[ptrnB]);

      sequenceCounter=0;



    }
    if(ers) ers=false;
  } 
  connectChannel(0);
  startConversion();

}
bool osc1state;
#define SIX_OCTAVES_POINT 255
void loop() { 
  if(!autoTune) pinMode(0,INPUT), OCR0A=0;
  else pinMode(0,OUTPUT);
  // pinMode(2,INPUT);
  bool newState=bitRead(PINB,4);
  if(newState && !osc1state){ //rising edge action

  }
  if(!newState && osc1state){ //falling edge falling edge action
    if(rec){

      if(sequenceLength[ptrnB]==0){
        if(sequenceCounter<63)  sequenceCounter++;
        else sequenceCounter=0,sequenceLength[ptrnB]=64;
      }
      else{
        if(sequenceCounter<(sequenceLength[ptrnB]-1))  sequenceCounter++;
        else sequenceCounter=0;
      }

      sequence[ptrnB][sequenceCounter]=note;
      //      coarseTuneValue=map(sequence[ptrnB][sequenceCounter],0,NUMBER_OF_NOTES,SIX_OCTAVES_POINT,0);
      coarseTuneValue=128;
      // coarseTuneValue=0;
    }


    else if(play){

      if(sequenceCounter<sequenceLength[ptrnB]-1)  sequenceCounter++;
      else sequenceCounter=0;

      if(sequence[ptrnB][sequenceCounter]==255) coarseTuneValue=128;
      else {


        coarseTuneValue=map(sequence[ptrnB][sequenceCounter],0,NUMBER_OF_NOTES,SIX_OCTAVES_POINT,0); //?? tuning
      }


    }


  }
  if(!play)  coarseTuneValue=128;
  if(ers){
    // sequence[ptrnB][sequenceCounter]=128;
  }
  else{

  }
  if(play){
    if(ptrnA) digitalWrite(2,LOW),pinMode(2,OUTPUT);
    else digitalWrite(2,HIGH),pinMode(2,OUTPUT);
  }
  else digitalWrite(2,LOW),pinMode(2,INPUT);
  osc1state=newState;
  OCR0B= coarseTuneValue;

}

ISR(TIMER1_COMPA_vect)  //audiorate interrupt
{
  /*
  lfoValue++;
   if(lfoFlop && lfoValue<200) bitWrite(PORTB,PINB2, 1);
   else bitWrite(PORTB,PINB2, 0);
   doReset=bitRead(PINB,PINB3);
   if(!lastDoReset && doReset) {
   lfoValue=0, lfoFlop=0;
   }
   lastDoReset=doReset;
   if(lfoValue==0){
   lfoFlop=!lfoFlop;
   
   bool newBit= bitRead(runglerByte,7) ;
   runglerByte=runglerByte<<1;
   if((analogValues[0]>>2)<150){
   newBit=newBit;
   }
   else if((analogValues[0]>>2)>162){
   newBit=TCNT0>>7;
   }
   else newBit=!newBit;
   bitWrite(runglerByte,0,newBit);
   runglerOut=0;
   bitWrite(runglerOut,0,bitRead(runglerByte,0));
   bitWrite(runglerOut,1,bitRead(runglerByte,3));
   bitWrite(runglerOut,2,bitRead(runglerByte,5));
   runglerOut=runglerMap[runglerOut];
   }
   if(lfoFlop) out =255-lfoValue;
   else out=lfoValue;
   OCR0B= constrain(out,0,255);
   OCR0A= runglerOut;
   */
  //coarseTuneValue++;
  //fineTuneValue++;

  //TCNT1 = 0; 


}

void setFrequency(int _freq){
  _freq=(2048-(_freq<<3))+20;
  uint8_t preScaler=_freq>>7;
  preScaler+=1; //*2
  for(uint8_t i=0;i<4;i++) bitWrite(TCCR1,CS10+i,bitRead(preScaler,i)); 
  uint8_t compare=_freq;
  bitWrite(compare,7,0);
  OCR1A=compare+128; 
}


// #### FUNCTIONS TO ACCES ADC REGISTERS
void init() {

  ADMUX  = 0;
  bitWrite(ADCSRA,ADEN,1); //adc enabled
  bitWrite(ADCSRA,ADPS2,1); // set prescaler
  bitWrite(ADCSRA,ADPS1,1); // set prescaler
  bitWrite(ADCSRA,ADPS0,1); // set prescaler
  bitWrite(ADCSRA,ADIE,1); //enable conversion finished interupt
  bitWrite(SREG,7,1);
  // prescaler = highest division
}


// channel 8 can be used to measure the temperature of the chip
void connectChannel(uint8_t number) {
  ADMUX &= (11110000);
  ADMUX |= number;  
}

void startConversion() {
  bitWrite(ADCSRA,ADSC,1); //start conversion
}

bool isConversionFinished() {
  return (ADCSRA & (1<<ADIF));
}

bool isConversionRunning() {
  return !(ADCSRA & (1<<ADIF));
}

uint16_t getConversionResult() {
  uint16_t result = ADCL;
  return result | (ADCH<<8);
}








