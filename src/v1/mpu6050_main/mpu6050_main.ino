#include <Wire.h>

#define CONSOLE_BUF_SIZE (80)
#define SAMPLE_SIZE (2 /* 16 bit */ * 3 /* X Y Z */)
#define SAMPLE_BUF (2)
#define MPU_addr (0x68)
#define MPU_reg_accel_config (0x1c)
#define MPU_reg_accel_out (0x3b)
#define MPU_reg_config (0x1a)
#define MPU_reg_fifo_en (0x23)
#define MPU_reg_fifo_count_h (0x72)
#define MPU_reg_fifo_r_w (0x74)
#define MPU_reg_int_status (0x3a)
#define MPU_reg_pwr_mgmt_1 (0x6b)
#define MPU_reg_signal_path_reset (0x68)
#define MPU_reg_smprt_div (0x19)
#define MPU_reg_user_ctrl (0x6a)
#define MPU_reg_whoami (0x75)

char HEX_DIGITS[] = "0123456789ABCDEF";
void (*g_bg_task)(int) = NULL;

void task_arun(int keep_running){
  static uint8_t sample_buf[SAMPLE_SIZE*SAMPLE_BUF];
  static byte sample_cnt = 0;
  static byte t = 0;
  if(!keep_running){
      //FIFO_EN = temp_fifo_en, xg_fifo_en, yg_fifo_en, zg_fifo_en, accel_fifo_en, slv2_fifo_en, slv1_fifo_en, slv0_fifo_en
      //Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_fifo_en); Wire.write(0x00 /* disable all */); Wire.endTransmission(true);
      //Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_user_ctrl); Wire.write(0x04 /*FIFO stop & reset*/); Wire.endTransmission(true);
      Serial.println("arun done");
      sample_cnt = 0;
      return;
  }

  Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_int_status); Wire.endTransmission(false); Wire.requestFrom(MPU_addr, 1, true);
  if(Wire.read() & 1 /*data_rdy_bit*/){
    Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_accel_out); Wire.endTransmission(false); Wire.requestFrom(MPU_addr, SAMPLE_SIZE, true);
    for(int i=0; i < SAMPLE_SIZE; i++){
      sample_buf[sample_cnt*SAMPLE_SIZE + i] = Wire.read();
    }
    sample_cnt++;
  }
  if(sample_cnt == SAMPLE_BUF){
    for(int a_count=0; a_count < SAMPLE_SIZE / 2; a_count++){
      int32_t a_avg = 0;
      for(int s_count=0; s_count < SAMPLE_BUF; s_count++){
        int pbuf = s_count*SAMPLE_SIZE + a_count * 2;
        a_avg += ((int16_t)sample_buf[pbuf] << 8) + sample_buf[pbuf+1];
      }
      a_avg /= SAMPLE_BUF;
      sample_buf[a_count*2] = a_avg >> 8;
      sample_buf[a_count*2 + 1] = a_avg;
    }
    Serial.print("0x");
    for(int i=0; i < SAMPLE_SIZE; i++){
      byte r = sample_buf[i]; 
      Serial.write(HEX_DIGITS[r >> 4]); 
      Serial.write(HEX_DIGITS[r & 0xf]);
    }
    Serial.println("");    
    sample_cnt = 0;
  }
  /*
  Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_fifo_count_h); Wire.endTransmission(false); Wire.requestFrom(MPU_addr, 2, true);
  uint16_t cnt = Wire.read(); cnt = cnt << 8 + Wire.read();
  if(cnt){
    int bytes_to_read = FIFO_ITEM_SIZE - buf_cnt;
    if(cnt < bytes_to_read) bytes_to_read = cnt;
    Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_fifo_r_w); Wire.endTransmission(false); Wire.requestFrom(MPU_addr, 1, true);
    //while(bytes_to_read--) 
    buf[buf_cnt++] = Wire.read();
  }else{
    Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_fifo_r_w); Wire.write(t++); Wire.endTransmission(true);
  }
  
  if(FIFO_ITEM_SIZE == buf_cnt){
    Serial.print("0x");
    for(int i=0; i < FIFO_ITEM_SIZE; i++){
      byte r = buf[i]; 
      Serial.write(HEX_DIGITS[r >> 4]); 
      Serial.write(HEX_DIGITS[r & 0xf]);
    }
    buf_cnt = 0;
    Serial.println("");
  }
  */
}
void execute_command(const char *command){
  Serial.println("");
  if(!strcmp(command, "")){
    
  }else if(!strcmp(command, "help") || !strcmp(command,"?")){
    Serial.println("Available commands:");
    Serial.println("? help: show this message");
    Serial.println("ainit: initialize accelerometer");
    Serial.println("aread: read accelerometer once");
    Serial.println("arun: run continous accelerometer reading");
  }else if(!strcmp(command, "ainit")){
    Serial.print("Initating accelerometer at 0x");Serial.println(MPU_addr, HEX);
    Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_whoami); Wire.endTransmission(false); Wire.requestFrom(MPU_addr, 1, true);
    byte resp = Wire.read();
    Serial.print("ID=");Serial.println(resp, HEX);
    if(resp == MPU_addr){
      Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_pwr_mgmt_1); Wire.write(0); Wire.endTransmission(true);
      // SIGNAL_PATH_RESET = reserved[5], gyro_reset, accel_reset, temp_reset
      Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_signal_path_reset); Wire.write(2 /*accel reset*/); Wire.endTransmission(true);
      //CONFIG = reserved[2], ext_sync_set[3], dlpf_cfg[3]
      // dlpf_cfg = 0:off, 1...6: low pass filter strength, 7: reserved
      //  6: 19ms, 5: 13.8ms, ... , 1: 2 ms
      Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_config); Wire.write(6/* 19ms per sample */); Wire.endTransmission(true);
      //SMPRT_DIV = SMPRT_DIV[8]
      //Rate = GyroRate(1kHz)/(1+SMPRT_DIV)
      Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_smprt_div); Wire.write(51 /* 1000ms/19ms - 1 */); Wire.endTransmission(true);
      Serial.println("OK");
    }else{
      Serial.println("ERROR");
    }
  }else if (!strcmp(command, "aread")){
    Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_accel_out); Wire.endTransmission(false); Wire.requestFrom(MPU_addr, 2*3, true);
    Serial.print("0x");
    for(int i=0; i < 2*3; i++){byte resp = Wire.read(); Serial.write(HEX_DIGITS[resp >> 4]); Serial.write(HEX_DIGITS[resp & 0xf]);}
    Serial.println("");
  }else if(!strcmp(command, "arun")){
      //FIFO_EN = temp_fifo_en, xg_fifo_en, yg_fifo_en, zg_fifo_en, accel_fifo_en, slv2_fifo_en, slv1_fifo_en, slv0_fifo_en
      //Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_fifo_en); Wire.write(0x08 /* accel_fifo_en */); Wire.endTransmission(true);
      //Wire.beginTransmission(MPU_addr); Wire.write(MPU_reg_user_ctrl); Wire.write(44 /*FIFO enable & reset*/); Wire.endTransmission(true);
      Serial.println("Press <Enter> to stop");
      g_bg_task = task_arun;
  }else{
    Serial.print("Unknown command: "); Serial.println(command);
  }
  Serial.print("?>");
}

void console_handle(){
  static int g_console_char_count = 0;
  static char g_console_chars[CONSOLE_BUF_SIZE+1];
  
  int char_new_count = Serial.available();
  while(char_new_count--){
    char new_in = Serial.read();
    if(new_in == '\r'){
      //Enter (CR) is end of line. So execute command from buffer.
      if(g_bg_task){
        g_bg_task(0);
        g_bg_task = NULL;
      }else{
        execute_command(g_console_chars);
      }
      g_console_char_count = -1;
    }else if(new_in == '\n'){
      //ignore new line
      g_console_char_count--;
    }else if(new_in == '\b'){
      if(g_console_char_count){
        g_console_char_count -= 2;
      }
      Serial.print(new_in);    
    }else if(g_console_char_count <= CONSOLE_BUF_SIZE) {
      g_console_chars[g_console_char_count] = new_in;
      Serial.print(new_in);
    }else{
      g_console_char_count--;
    }
    g_console_chars[++g_console_char_count] = '\0';
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("\r\nArduino accelerometer sketch. Type <help> to get help");
  Wire.begin();
  execute_command("");
}

void loop() {
  // put your main code here, to run repeatedly:
  console_handle();
  if(g_bg_task){
    g_bg_task(1);
  }
}
