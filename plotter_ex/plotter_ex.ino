float sample_time;
float sim_time = 0;

uint32_t start_time, end_time;
uint32_t MicrosSampleTime;

float sin_value;
float cos_value;

uint8_t cnt1 = 0;
uint8_t cnt2 = 0;


void setup() {
  // put your setup code here, to run once:
  int i;
  Serial.begin(115200);
  sample_time = 0.02;
  MicrosSampleTime = (unsigned long)(sample_time*1e6);
}

void loop() {
  // put your main code here, to run repeatedly:
  start_time = micros();
  end_time = start_time + MicrosSampleTime;

  sim_time += sample_time;

  cnt1++;
  cnt2--;
  sin_value = sin(sim_time);
  cos_value = cos(sim_time*3.0);

  Serial.print(cnt1);
  Serial.print(" ");
  Serial.print(cnt2);
  Serial.print(" ");
  Serial.print(sin_value);
  Serial.print(" ");
  Serial.println(cos_value);

  
    while( !((end_time - micros()) & 0x80000000));
}
