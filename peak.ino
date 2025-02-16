#include <SPI.h>
#include <vector>
#include <ArduinoSTL.h>
#include <cmath>
// de SQ3SWF 2019

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
std::vector<int> Mag(datasize,0);
std::vector<int> Pha(datasize,0);
int medianMag;
int medianPha;

void prepare_registers() {

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

  prepare_registers();
  updateAllRegisters();
}


void fastSweep() { //This function is a fast sweep method algorithm
    time_now = micros();
    unsigned long long low = 49970000;
    unsigned long long high = 50070000;
    int current = low;
    const int incr = 1000; //Minimum 19 Hz
    std::vector<int> allMag(((high-low)/incr)+100,0); //Create list in the size of all frequency points in order to find the peak point
    std::vector<double> allFreq(((high-low)/incr)+100,0);

    allMag.clear(); //Initialize all vectors
    allFreq.clear();
    Mag.clear();
    Pha.clear();

    //First Level Sweep
    while (current < high) {  //check if sweep is still not over
        setFrequency(current);
        double freq1 = (double (INT) + (double(FRAC) /double(MOD)))*double(pfd_freq)/double(1<<rf_div_sel);

        while (digitalRead(input_pin)!=HIGH){ // wait for LOCK DETECT
            delayMicroseconds(10);
        }

        // READ AD8302
        for (int i = 0; i < 7; i++) {
            val1 = analogRead(AD8302_Mag_in);
            val2 = analogRead(AD8302_Pha_in);
            Mag.push_back(val1);
            Pha.push_back(val2);
            delayMicroseconds(105);
        }

        medianMag = Median(Mag);
        //medianPha = Median(Pha);
        //SerialUSB.println(String(medianMag)+" " + String(medianPha)+ " " + String(freq1));
        allFreq.push_back(freq1);
        allMag.push_back(medianMag);

        Mag.clear();
        Pha.clear();

        current+=incr;
    }
    //Second Level Sweep
    // After finding the peak frequency index , we are making a new sweep between index+1 and index-1, this time sweep increment is smallest step possible (19Hz) so we can detect the peak.
    int index1 = findPeakIndex(allMag);
    allMag.clear();
    double current2 = allFreq[index1 - 1];
    double high2 = allFreq[index1 + 1] + 1;
    allFreq.clear();
    setFrequency(current2);

    while (current2 < high2){
        setFrequency(current2);
        current2+=19;
        while (digitalRead(input_pin)!=HIGH){ // wait for LOCK DETECT
            delayMicroseconds(10);
        }
        // READ AD8302
        delayMicroseconds(105);
        for (int i = 0; i < 9; i++) {
            val0 = analogRead(AD8302_Mag_in);
            Mag.push_back(val0);
            delayMicroseconds(100);
        }
        medianMag = Median(Mag);
        allMag.push_back(medianMag);
        allFreq.push_back(current2);
        Mag.clear();
    }
    index1 = findPeakIndex(allMag);
    time_now2 = micros();
        
    //This part prints the information
    SerialUSB.println("Peak Frequency detected at : "+ String(allFreq[index1]) + " Time elapsed (microseconds): " + String ((time_now2-time_now)));
    time_now = micros();
}

double Median(std::vector<int> v) { 
    int n = datasize; 
    std::sort(v.begin(), v.end()); 
    return v[n / 2]; 
}

void sendRegisterToAdf(uint16_t reg_id) {
  
  digitalWrite(slave_select_pin, LOW);
  delay(1);
  SPI.transfer((uint8_t)(reg[reg_id] >> 24));
  SPI.transfer((uint8_t)(reg[reg_id] >> 16));
  SPI.transfer((uint8_t)(reg[reg_id] >> 8));
  SPI.transfer((uint8_t)(reg[reg_id]));
  
  digitalWrite(slave_select_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(slave_select_pin, LOW); 
  delayMicroseconds(10); 
  digitalWrite(slave_select_pin, HIGH);
}

void updateAllRegisters() {
  for(int i=5; i>=0; i--) {
    sendRegisterToAdf(i);
  }
}

int findPeakIndex(std::vector<int> vec) {
    if (!vec.empty()) {
        int max_value = vec[0]; // Assume the first element is the maximum
        size_t max_index = 0;

        // Loop through the vector to find the maximum value and its index
        for (size_t i = 1; i < vec.size(); ++i) {
            if (vec[i] > max_value) {
                max_value = vec[i];
                max_index = i;
            }
        }
        return max_index;
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

  // ADC -> ADC_MR |= 0x80;
  // ADC -> ADC_CR = 2;                                               // ADC Control Register (ADC_CR) is set to 2 (START).
  // ADC -> ADC_CHER = (1<<AD8302_Mag_in) | (1<<AD8302_Pha_in);       // ADC Channel Enable Register (ADC_CHER) is set to ...11000000 to enable CH7 and CH6 which corresponds to A0 and A1 on Arduino DUE Board.
  analogReadResolution(12); 
  // ADC -> ADC_CHER = 0x80;
}
;

void loop() {
  fastSweep();
}
