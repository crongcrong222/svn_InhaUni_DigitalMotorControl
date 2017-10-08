//pin A7 (PA2) 가변저항 테스트
//pin A6 (PA3) 가변저항 테스트


int cnt = 0;
int data_in;        // port 값을 읽어들여 저장할 변수
char str[64];       // message를 담기 위한 string data
signed int data[2];

void configure_adc();

void WordOutput();



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pmc_enable_periph_clk(ID_PIOD);
  // PIOD clock 활성화

  PIOD->PIO_PER = 0x00000011;     //pin 25 (PD0)의 PIO를 enable
  PIOD->PIO_IDR = 0x00000011;     //interrupt disable
  PIOD->PIO_IFER = 0x00000001;    //input filter enable register
  PIOD->PIO_ODR = 0x00000001;     //PD0을 입력으로 설정

  PIOD->PIO_PER = 0x00000010;     //pin14 를 출력으로 설정
  PIOD->PIO_OWER = 0x00000010;    //Output Write Enable 설정해서
                                  //PIO_ODSR에 data를 쓸 수 있도록 함  
  PIOD->PIO_ODSR = 0x00000000;
  
  
  configure_adc();
}

void loop() {
  // put your main code here, to run repeatedly:
  adc_start(ADC);

  while((ADC->ADC_ISR & ADC_ISR_EOC0) != ADC_ISR_EOC0);
  data[0] = adc_get_channel_value(ADC, ADC_CHANNEL_0) - 2600;
  /*
   * 원래 0~약 4000까지 가변하지만 속도 +- 표현을 위해
   * - 2600 정도로 조절했음
   */

  while((ADC->ADC_ISR & ADC_ISR_EOC1) !=ADC_ISR_EOC1);
  data[1] = adc_get_channel_value(ADC, ADC_CHANNEL_1) - 2600;


  /*
   *  data[0]과 data[1]은 각 두 개의 가변 저항 data
   */
  Serial.print(data[0]);
  Serial.print(" ");
  Serial.println(data[1]);
  /*
   * 시리얼 플로터로 나타내기 위해서는 항상 마지막 데이터가
   * \n으로 끝나야함
   * println은 마지막에 \n이 출력됨
   */
  delay(20);
}

void configure_adc()
{
  pmc_enable_periph_clk(ID_ADC);      //ADC에 clock 활성화
  adc_disable_all_channel(ADC);
  /*
   *  adc.h에 정의 되어 있는 상수
   *  the max adc sample freq definition
   *  #define ADC_FREQ_MAX 20000000
   *  the min adc sample freq definition
   *  #define ADC_FREQ_MIN  1000000
   *  the normal adc startup time
   *  #define ADC_STARTUP_ NORM  40
   *  the fast adc startup time
   *  #define ADC_STARTUP_FAST   12
   */

   adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
   adc_configure_timing(ADC, 1, ADC_SETTLING_TIME_3, 1);

   adc_set_resolution(ADC, ADC_12_BITS);

   adc_enable_channel(ADC, ADC_CHANNEL_0);
   adc_enable_channel(ADC, ADC_CHANNEL_1);
}

