#define CONSOLE_BUF_SIZE (80)


void execute_command(const char *command){
  Serial.println("");
  if(!strcmp(command, "")){
    
  }else if(!strcmp(command, "help") || !strcmp(command,"?")){
    Serial.println("Available commands:");
    Serial.println("? help: show this message");
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
    if(new_in == '\n'){
      //ignore new line
      g_console_char_count--;
    }else if(new_in == '\r'){
      //Enter (CR) is end of line. So execute command from buffer.
      execute_command(g_console_chars);
      g_console_char_count = -1;
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
  execute_command("");
}

void loop() {
  // put your main code here, to run repeatedly:
  console_handle();
}
