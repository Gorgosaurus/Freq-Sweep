#include <SPI.h>
#include <vector>
#include <ArduinoSTL.h>
#include <cmath>
//  SQ3SWF 2019

#define SHL(x,y) ((uint32_t)1<<y)*x

#define REF_CLK 25000000
uint8_t slave_select_pin = 47;   

int input_pin = 40; 
int AD8302_Mag_in = A2 ;   //  Variable to select appropriate channel of ADC corresponding to A2
int AD8302_Pha_in = A3  ;  //  Variable to select appropriate channel of ADC corresponding to A3
int val0=0;  
int val1=0;
int val2=0;
int i=5; 
unsigned long time_now=0;
unsigned long time_now2=0;
unsigned long a=0;
unsigned long long frequency = 50000000;
const int supplyPin = A5;
   
// Register 0:
uint16_t INT=0;
uint16_t FRAC=0;  

// Register 1:
uint8_t phase_adj=0;   
uint8_t prescaler = 0;
uint16_t phase = 1;
uint16_t MOD = 4095;

// Register 2: 
uint8_t low_noise_spur=3;
uint8_t muxout=6;
uint8_t ref_doubler=1;
uint8_t rdiv2 = 0;
uint16_t r_counter = 10;
uint8_t dbl_buf;
uint8_t charge_pump_current = 0b111;
uint8_t ldf=1; uint8_t ldp=0;
uint8_t pd_polarity = 1;
uint8_t powerdown;
uint8_t cp_three_state; 
uint8_t counter_reset;

// Register 3:
uint8_t band_mode_clksel=1;
uint8_t abp; 
uint8_t chg_cancel=0;
uint8_t csr=0;
uint8_t clkdiv_mode=0;
uint16_t clock_divider = 150;

// Register 4:
uint8_t feedback_sel =1;
uint8_t rf_div_sel = 6;   // 0 = /1, 1=/2, 2=/4 ...
uint8_t band_select_clkdiv = 50;
uint8_t vco_pwrdown = 0;
uint8_t mtld = 1;
uint8_t aux_outsel = 0;
uint8_t aux_outena;
uint8_t aux_pwr;
uint8_t rf_ena =1 ; // 0 - output disabled
uint8_t out_pwr = 3; // 0 - min, 3 - max

// Register 5:
uint8_t ld_pinmode = 1;
uint32_t reg[6] = {0,0,0,0,0,0};
uint32_t pfd_freq = (REF_CLK*(1.0+ref_doubler))/(r_counter*((1.0+rdiv2)));

//For Median Function
const int datasize = 11;
std::vector<double> Mag(datasize,0);
std::vector<double> Pha(datasize,0);

void prepare_registers() { //This function is prepared according to ADF4351 Datasheet

reg[0] = SHL(INT, 15) | SHL(FRAC, 3) | 0b000;
reg[1] = SHL(phase_adj, 28)  | SHL(prescaler, 27)  | SHL(phase, 15)  | SHL(MOD, 3) | 0b001;
reg[2] = SHL(low_noise_spur, 29) | SHL(muxout, 26) | SHL(ref_doubler, 25) | SHL(rdiv2, 24) | SHL(r_counter, 14) \
          | SHL(dbl_buf, 13) | SHL(charge_pump_current, 9) | SHL(ldf, 8) | SHL(ldp, 7) | SHL(pd_polarity, 6) \
          | SHL(powerdown, 5) | SHL(cp_three_state, 4) | SHL(counter_reset, 3) | 0b010;
reg[3] = SHL(band_mode_clksel, 23) | SHL(abp, 22) | SHL(chg_cancel, 21) | SHL(csr, 18) | SHL(clkdiv_mode, 15) \
          | SHL(clock_divider, 3) | 0b011;
reg[4] = SHL(feedback_sel, 23) | SHL(rf_div_sel, 20) | SHL(band_select_clkdiv, 12) | SHL(vco_pwrdown, 9) \
          | SHL(mtld, 10) | SHL(aux_outsel, 9) | SHL(aux_outena, 8) | SHL(aux_pwr, 6) | SHL(rf_ena, 5) | SHL(out_pwr, 3) | 0b100;
reg[5] = SHL(ld_pinmode, 22) | SHL(0b11, 19) | 0b101;
}

void setFrequency(unsigned long long freq) { //This function calculates INT and FRAC values for their corresponding Frequency value.
  int remainder;
  unsigned long long frequency1 = freq;

  INT = ((frequency1 * (1 << rf_div_sel)) / pfd_freq);
  remainder = (frequency1 * (1 << rf_div_sel)) % pfd_freq;
  FRAC = static_cast<int>(round((double(remainder)*double(MOD)) / double(pfd_freq)));

  digitalWrite(slave_select_pin, LOW);
  delay(1);
  prepare_registers();
  updateAllRegisters();
  delay(1);
  digitalWrite(slave_select_pin, HIGH);
}

void Sweep() { //This function is a fast sweep method algorithm
    unsigned long long low = 49960000;
    unsigned long long high = 50080000;
    int current = low;
    const int incr = 19; //Minimum 19 Hz
    Mag.clear();
    Pha.clear();

    while (current < high) {  //check if sweep is still not over
        setFrequency(current);
        double freq1 = (double (INT) + (double(FRAC) /double(MOD)))*double(pfd_freq)/double(1<<rf_div_sel);
     while (digitalRead(input_pin)!=HIGH){ // wait for LOCK DETECT
   
            delayMicroseconds(1);
        }

        // READ AD8302
        for (int i = 0; i < 7; i++) {
            val1 = analogRead(AD8302_Mag_in);
            val2 = analogRead(AD8302_Pha_in);
            Mag.push_back(val1);
            Pha.push_back(val2);
            delayMicroseconds(100);
        }

        SerialUSB.println(String(Median(Mag))+" " + String(Median(Pha))+ " " + String(freq1));

        Mag.clear();
        Pha.clear();

        current+=incr;
    }
}

double Median(std::vector<double> v) { 
    int n = datasize; 
    std::sort(v.begin(), v.end()); 
    return (double)v[n / 2]; 
}

void sendRegisterToAdf(uint16_t reg_id) {
  
  digitalWrite(slave_select_pin, LOW);
  delay(1);
  SPI.transfer((uint8_t)(reg[reg_id] >> 24));
  SPI.transfer((uint8_t)(reg[reg_id] >> 16));
  SPI.transfer((uint8_t)(reg[reg_id] >> 8));
  SPI.transfer((uint8_t)(reg[reg_id]));
  
  digitalWrite(slave_select_pin, HIGH);
  delay(1);
  digitalWrite(slave_select_pin, LOW);  
  delay(1);
}

void updateAllRegisters() {
  for(int i=5; i>=0; i--) {
    sendRegisterToAdf(i);
  }
}

void setup() {
  pinMode (supplyPin, OUTPUT);
  pinMode (input_pin,INPUT); 
  pinMode (slave_select_pin, OUTPUT);
  digitalWrite(slave_select_pin, LOW);
  
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.begin();

  Serial.begin(115200);

  delayMicroseconds(1);

  if(frequency >= 2200000000) rf_div_sel = 0;
  if(frequency < 2200000000) rf_div_sel = 1;
  if(frequency < 1100000000) rf_div_sel = 2;
  if(frequency < 550000000) rf_div_sel = 3;
  if(frequency < 275000000) rf_div_sel = 4;
  if(frequency < 137500000) rf_div_sel = 5;
  if(frequency < 68750000) rf_div_sel = 6;
  INT = ((frequency*(1<<rf_div_sel))/pfd_freq);

  prepare_registers();
  updateAllRegisters(); 
  analogReadResolution(12); 
}
;

void loop() {
  Sweep();

}
