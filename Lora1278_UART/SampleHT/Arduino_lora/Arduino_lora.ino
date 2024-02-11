/*
 * Demo project tren Arduino de giao tiep voi board Lora
 * Su dung board Due hoac ATmega2560
 * Hardware connnection:
 *     Due/ATmega2560         Board Lora
 *        TX1      -------      RX
 *        RX1      -------      TX
 *        2        -------      RST
 *        3.3V     -------      3.3V
 *        GND      -------      GND
 */


#define COMMAND_TIMEOUT_MS  300   //thoi gian doi response tu board Lora sau khi send AT command
#define MAX_TX_BUF_SIZE     300   //max buffer size de nhan data tu Serial Monitor window
#define MAX_RESPONSE_SIZE   100   //max buffer size de nhan data tu board Lora
#define RESET_PIN 2               //su dung pin 2 tren board Arduino de dieu khien chan reset cua board Lora

typedef enum {
  CHECKING_CONNECTION,
  SETTING_LORA,
  SENDING_AT_CMD,
  SENDING_DATA,
  RECEIVING_DATA,
}control_state_t;

control_state_t Control_State;
uint8_t Is_Receiving;
char Tx_Data[MAX_TX_BUF_SIZE];
int Tx_Len;
char AT_Response[MAX_RESPONSE_SIZE];
uint8_t Response_Len;
uint8_t Is_Waiting_Response;

//Goi de reset board Lora
void resetLora() {
  digitalWrite(RESET_PIN, HIGH);
  delay(200);
  digitalWrite(RESET_PIN, LOW);
  delay(200);
  digitalWrite(RESET_PIN, HIGH);
  delay(200);  
}

void setup() {
  //Su dung UART1 tren board Arduino de giao tiep voi board Lora
  Serial1.begin(115200);
  //Su dung UART0 de print log va nhan tx data tu Serial Monitor
  Serial.begin(115200);
  //Reset board Lora
  resetLora();
}

//Goi de gui lenh AT command va doi response tu board Lora
bool Send_And_Wait(String cmd, String response) {
  //Gui AT command
  Serial1.flush();
  Serial1.print(cmd);
  
  //Delay de cho response
  delay(COMMAND_TIMEOUT_MS);
  
  //Kiem tra co nhan dc response khong
  if(!Serial1.available()) {
    return false;
  }
  
  //Doc response data nhan duoc tu board Lora
  memset(AT_Response, 0, MAX_RESPONSE_SIZE);
  while(Serial1.available() > 0)
  {
    AT_Response[Response_Len++] = Serial1.read();
  }
  printLog("LoraResponse:" + String(AT_Response));
  
  //Kiem tra response co giong nhu mong doi khong
  if((String)AT_Response == response) {
    return true;
  }
  else {
    return false;
  }
}


void loop() {
  //Kiem tra va doc data gui tu Serial Monitor window
  while(Serial.available() > 0) {
    char input = Serial.read();
    Tx_Data[Tx_Len++] = input;
  }

  //Kiem tra va doc response gui tu board Lora
  while (Serial1.available() > 0) {
    //Is_Receiving = 1;
    uint8_t data = Serial1.read();
    AT_Response[Response_Len++] = data;
  }  
  
  //Thuc hien state machine 
  switch(Control_State) {
    /*
     * O trang thai CHECKING_CONNECTION, chung ta se gui "AT\r\n" va doi nhan "OK\r\n" tu board Lora de chac chan ket noi OK
     * Sau khi kiem tra, chung ta se chuyen sang trang thai SETTING_LORA
     */
    case CHECKING_CONNECTION:
        printLog("Checking connection...");
        if((String)AT_Response != "OK\r\n") {
          serialSendString("AT\r\n");
        }
        else
        {
          Control_State = SETTING_LORA;
        }
        break;
    /*
     * O trang thai SETTING_LORA, chung ta co the gui cac AT command de thay doi cau hinh cua board Lora
     * O demo nay, chung ta khong can thay doi gi ca, va chi can su dung cau hinh default thoi
     * Sau khi xong viec thay doi cau hinh board Lora, chung ta se chuyen sang trang thai RECEIVING_DATA de xu ly 
     *  cac data nhan duoc (tu board Lora hay tu user trong Serial Monitor window)
     */        
    case SETTING_LORA:
        printLog("Setting Lora...");
        //do lora setup from here
        
        //done setup lora device, move to Rx state
        printLog("Type any text to send:");
        Control_State = RECEIVING_DATA;
        Tx_Len = 0;
        Response_Len = 0;
        printLog("Send AT+RECV");
        Send_And_Wait("AT+RECV=zz\r\n", "OK\r\n");
        resetResponseBuffer();
        break;
    /*
     * O trang thai SENDING_AT_CMD, chung ta se gui AT command (dc nhan tu Serial Monitor window) xuong board Lora
     * Sau khi send, chung ta se doi response tu board Lora va print ra Serial Monitor window
     * Cuoi cung thi chung ta tro lai trang thai RECEIVING_DATA de cho xu ly data tiep theo
     */            
    case SENDING_AT_CMD:
        if(Is_Waiting_Response == 0) {
          printLog("Send AT cmd:" + (String)Tx_Data);
          serialSendString((String)Tx_Data + "\r\n");
          Is_Waiting_Response = 1;
        }
        if(Response_Len) {
          Is_Waiting_Response = 0;
          printLog("LoraResponse:" + (String)AT_Response);
          Control_State = RECEIVING_DATA;
          resetTxBuffer();
          resetResponseBuffer();
          printLog("Type any text to send:");
        }
        break;
    /*
     * O trang thai SENDING_DATA, chung ta se gui Tx data(dc nhan tu Serial Monitor window) xuong board Lora de truyen di
     * Sau khi send, chung ta se tro lai trang thai RECEIVING_DATA de cho xu ly data tiep theo
     */               
    case SENDING_DATA:
        if(Tx_Len)
        {
          printLog("Sending data: " + String(Tx_Data));
          serialSendString("AT+SEND=");       //Send Tx command
          serialSendBuffer((char *)&Tx_Len, 1); //Tx len
          serialSendBuffer(Tx_Data, Tx_Len);    //Tx data
          Send_And_Wait("\r\n", "OK\r\n");             //Ending characters
          resetTxBuffer();
          resetResponseBuffer();
        }
        else
        {
          printLog("Type any text to send:");
          Control_State = RECEIVING_DATA;
        }
        break;
    /*
     * O trang thai RECEIVING_DATA, chung ta se cho nhan data tu board Lora hoac tu Serial Monitor window
     * Neu nhan duoc AT command can gui tu Serial Monitor window (Tx_Len > 0 va 3 ky tu dau la "AT+") thi se chuyen sang trang thai SENDING_AT_CMD de gui AT command xuong board Lora
     * Neu nhan duoc Tx data can gui tu Serial Monitor window (Tx_Len > 0 va kg co 3 ky tu dau la "AT+") thi se chuyen sang trang thai SENDING_DATA de Tx data xuong board Lora
     * Neu nhan duoc response la "RX TIMEOUT" thi tu gui lenh AT command se tiep tuc vao trang thai Rx cho board Lora
     * Neu nhan duoc response ma ky tu dau tien la '>' chinh la Rx data ma board Lora nhan dc thi print "Rx: <data>" ra Serial Monitor window
     * Neu nhan duoc cac response khac thi chi print ra Serial Monitor window binh thuong
     */            
    case RECEIVING_DATA:
        if(Tx_Len) {
          if(memcmp(Tx_Data, "AT+", 3) == 0) {
            Response_Len = 0;
            Control_State =  SENDING_AT_CMD;
          }
          else {
            Control_State = SENDING_DATA;
          }
        }
        if(Response_Len)
        {
          if(AT_Response[0] == '>') {
            printLog("Rx:" + (String)&AT_Response[1]);
          }
          else if ((String)AT_Response == "RX TIMEOUT\r\n") {
            printLog("LoraResponse:" + (String)AT_Response);
            Send_And_Wait("AT+RECV=zz\r\n", "OK\r\n");
          } else {
            printLog("LoraResponse:" + (String)AT_Response);
          }
          resetResponseBuffer();
        }
        
        break;
    default: break;
  }

  delay(100);
  
}

void resetResponseBuffer() {
  Response_Len = 0;
  memset(AT_Response, 0, MAX_RESPONSE_SIZE);
}

void resetTxBuffer() {
  Tx_Len = 0;
  memset(Tx_Data, 0, MAX_TX_BUF_SIZE);
}

void serialSendString(String str) {
  Serial1.flush();
  Serial1.print(str);
}

void serialSendBuffer(char *buf, int len) {
  Serial1.flush();
  Serial1.write(buf, len);
}

void printLog(String message) {
  Serial.println(message);
}



