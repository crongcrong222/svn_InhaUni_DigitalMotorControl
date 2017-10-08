void configure_encoder_counter();
char str[64];
uint32_t cnt1, cnt2, start_count,result, re_count;
float sample_time;
uint32_t MicrosSampleTime;
uint32_t start_time, end_time;


void setup()
{
  Serial.begin(115200);
  configure_encoder_counter();
  sample_time = 0.01;
  MicrosSampleTime = (uint32_t)(sample_time*1e6); //델타 t(sample time)
                                                  // micro second 단위
}

void loop()
{
  delay(100);
  start_count = cnt2;                  // 시작 펄스수
  cnt2 = TC2->TC_CHANNEL[0].TC_CV;     // 끝 펄스수
  //sprintf(str,"cnt1 = %u, cnt2 = %u\n", cnt1, cnt2);  
  //start_time = micros();
  //end_time = start_time + MicrosSampleTime;
  re_count = cnt2-start_count;        //변한 펄스 수
  result = (((re_count*2*PI)/64)/MicrosSampleTime)*1e6;
  //  rad/sec
  Serial.println(result);
  re_count = 0;
  while(!((end_time - micros()) & 0x80000000));
  end_time += MicrosSampleTime;
  //Serial.print(str);
}
void configure_encoder_counter()
{
  PIOC->PIO_PDR = (PIO_PC25)|(PIO_PC26);
  PIOC->PIO_ABSR = (PIO_PC25)|(PIO_PC26);

  pmc_enable_periph_clk(ID_TC6);
  TC2->TC_CHANNEL[0].TC_CMR = 5;
  TC2->TC_BMR = (1<<9)|(1<<8)|(1<<12)|(1<<19)|(30<<20);
  TC2->TC_CHANNEL[0].TC_CCR = 5;

}

