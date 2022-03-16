volatile unsigned long timerCounts;
volatile boolean counterReady;

// counting routine
unsigned long OVFcount;
unsigned int timerTicks;
unsigned int timerPeriod;

#define vdd 26

void startCounting (unsigned int ms){

  counterReady = false;
  timerPeriod = ms; // how many counts in 1 ms
  timerTicks = 0;
  OVFcount = 0; 
  
  pmc_enable_periph_clk((uint32_t)TC1_IRQn);
  pmc_enable_periph_clk((uint32_t)TC2_IRQn);

  // TC0 ch1
  TC0->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_XC0 | // XC0 -> external clock pin D22
                              TC_CMR_WAVSEL_UP  | // normal mode until OVF = 2^32 - 1
                              TC_CMR_EEVTEDG_RISING;  // external event -> rising edge

  // TC0 ch2
  TC0->TC_CHANNEL[2].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |  // timer_clock1 -> 84 MHz / 2 = 42 MHz (23.8 ns by tick)
                              TC_CMR_WAVSEL_UP_RC;  // RC comparator

  // set RC to interrupt every 1 ms = 42000 * 23.8 ns 
  //TC_SetRC(TC0, 2, 42000);
  TC_SetRC(TC0, 2, 42041); // better for 10 MHz frequency
  
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_COVFS;
  TC0->TC_CHANNEL[1].TC_IDR = ~TC_IER_COVFS;

  TC0->TC_CHANNEL[2].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[2].TC_IDR = ~TC_IER_CPCS;
  
  NVIC_DisableIRQ(TC1_IRQn);
  NVIC_DisableIRQ(TC2_IRQn);
  NVIC_ClearPendingIRQ(TC1_IRQn);
  NVIC_ClearPendingIRQ(TC2_IRQn);
  //NVIC_SetPriority(TC2_IRQn, (uint32_t)PRIOR_TIMER);
  NVIC_EnableIRQ(TC1_IRQn);
  NVIC_EnableIRQ(TC2_IRQn);

  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
  TC0->TC_CHANNEL[2].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

// TC0 channel 1
void TC1_Handler(){
  TC_GetStatus(TC0, 1);
  ++OVFcount; // OVF counter
}

// TC0 channel 2
void TC2_Handler(){
  TC_GetStatus(TC0, 2);
  unsigned long timer1count;
  timer1count = TC0->TC_CHANNEL[1].TC_CV;
  unsigned long OVFcopy = OVFcount;

  // verify if time period is exceeded
  if (++timerTicks < timerPeriod){
    return;
  }

  // if one OVF is lost
  //if (((status & TC_SR_COVFS) == TC_SR_COVFS) && timer1count < 4294967296){
   // OVFcopy++;
  //}

  TC0->TC_CHANNEL[1].TC_CMR = 0;
  TC0->TC_CHANNEL[2].TC_CMR = 0;
  
  TC0->TC_CHANNEL[1].TC_IDR |= TC_IER_COVFS;
  TC0->TC_CHANNEL[2].TC_IDR |= TC_IER_CPCS;

  
  // counter
  timerCounts = (OVFcopy << 32) + timer1count; // OVFcopy << 32 = OVFcopy*2^32
  counterReady = true;
}

void setup() {
  Serial.begin(115200);
  //Serial.println("Frequency meter");

  pinMode(vdd, OUTPUT);
  digitalWrite(vdd, HIGH);
}

void loop() {

  if (Serial.available()){
    float msj = Serial.read();
    if (msj == '0'){
      Serial.println("0");
      digitalWrite(vdd, HIGH);
    }
    if (msj == '1'){
      Serial.println("1");
      digitalWrite(vdd, LOW);
      while(1){
        startCounting(1000); // time period in ms
      while (!counterReady){
      }

       // frequency in Hz
      float freq = (timerCounts*1000.00)/ timerPeriod;

      Serial.print("Frequency: ");
      Serial.print((unsigned long) freq);
      Serial.println(" Hz");
      }
    }
  }

  delay(200);
}
