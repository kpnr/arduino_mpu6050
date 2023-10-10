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

void println(const char *s){
  Serial.println(s);
}

void print(const char *s){
  Serial.print(s);
}

void MPU_read_request(int reg, int count){
  Wire.beginTransmission(MPU_addr); Wire.write(reg); Wire.endTransmission(false); Wire.requestFrom(MPU_addr, count, true);
}

void MPU_write(int reg, int value){
  Wire.beginTransmission(MPU_addr); Wire.write(reg); Wire.write(value); Wire.endTransmission(true); 
}

void task_arun(int keep_running){
  static uint8_t sample_buf[SAMPLE_SIZE*SAMPLE_BUF];
  static byte sample_cnt = 0;
  static byte t = 0;
  if(!keep_running){
      println("arun done");
      sample_cnt = 0;
      return;
  }
  MPU_read_request(MPU_reg_int_status, 1);
  if(Wire.read() & 1 /*data_rdy_bit*/){
    MPU_read_request(MPU_reg_accel_out, SAMPLE_SIZE);
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
    print("0x");
    for(int i=0; i < SAMPLE_SIZE; i++){
      byte r = sample_buf[i]; 
      Serial.write(HEX_DIGITS[r >> 4]); 
      Serial.write(HEX_DIGITS[r & 0xf]);
    }
    println("");    
    sample_cnt = 0;
  }
}
void execute_command(const char *command){
  Serial.println("");
  if(!strcmp(command, "")){
    
  }else if(!strcmp(command, "help") || !strcmp(command,"?")){
    println("Available commands:");
    println("? help: show this message");
    println("ainit: initialize accelerometer");
    println("aread: read accelerometer once");
    println("arun: run continous accelerometer reading");
  }else if(!strcmp(command, "ainit")){
    print("Initating accelerometer at 0x");Serial.println(MPU_addr, HEX);
    MPU_read_request(MPU_reg_whoami, 1);
    byte resp = Wire.read();
    print("ID=");Serial.println(resp, HEX);
    if(resp == MPU_addr){
      MPU_write(MPU_reg_pwr_mgmt_1, 0);
      // SIGNAL_PATH_RESET = reserved[5], gyro_reset, accel_reset, temp_reset
      MPU_write(MPU_reg_signal_path_reset, 2 /*accel reset*/);
      //CONFIG = reserved[2], ext_sync_set[3], dlpf_cfg[3]
      // dlpf_cfg = 0:off, 1...6: low pass filter strength, 7: reserved
      //  6: 19ms, 5: 13.8ms, ... , 1: 2 ms
      MPU_write(MPU_reg_config, 6/* 19ms per sample */);
      //SMPRT_DIV = SMPRT_DIV[8]
      //Rate = GyroRate(1kHz)/(1+SMPRT_DIV)
      MPU_write(MPU_reg_smprt_div,51 /* 1000ms/19ms - 1 */);
      println("OK");
    }else{
      println("ERROR");
    }
  }else if (!strcmp(command, "aread")){
    MPU_read_request(MPU_reg_accel_out, 2*3);
    print("0x");
    for(int i=0; i < 2*3; i++){byte resp = Wire.read(); Serial.write(HEX_DIGITS[resp >> 4]); Serial.write(HEX_DIGITS[resp & 0xf]);}
    println("");
  }else if(!strcmp(command, "arun")){
    println("Press <Enter> to stop");
    g_bg_task = task_arun;
  }else{
    print("Unknown command: "); println(command);
  }
  print("?>");
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
      Serial.write(new_in);    
    }else if(g_console_char_count <= CONSOLE_BUF_SIZE) {
      g_console_chars[g_console_char_count] = new_in;
      Serial.write(new_in);
    }else{
      g_console_char_count--;
    }
    g_console_chars[++g_console_char_count] = '\0';
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  print("\r\nArduino accelerometer sketch. Type <help> to get help");
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
