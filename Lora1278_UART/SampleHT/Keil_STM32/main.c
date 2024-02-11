
#include "stm32f10x.h"
#include "main.h"
#include "stdbool.h"
#include "stdint.h"
#include <string.h>

#define NULL  0
typedef enum {
  CHECKING_CONNECTION,
  SETTING_LORA,
  SENDING_AT_CMD,
  SENDING_DATA,
  RECEIVING_DATA,
}control_state_t;

control_state_t Control_State;
uint8_t Is_Receiving;
uint8_t Tx_Data[MAX_TX_BUF_SIZE];
uint8_t Tx_Len;
uint8_t AT_Response[MAX_RESPONSE_SIZE];
uint8_t Response_Len;
uint8_t Is_Waiting_Response;
volatile bool Is_Response_Avail;
volatile uint8_t Is_Receiving_Done;

#define CPU_CLK_FREQ_HZ     72000000
#define LOOP_COUNT_FOR_MS (CPU_CLK_FREQ_HZ/10000)
void delay(uint32_t ms) //roughly blocking delay
{  
  while(ms--)
  {
    for(uint32_t i=0; i < LOOP_COUNT_FOR_MS; i++);
  }
}

void Board_Init()
{
  GPIO_InitTypeDef 	GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  //init UART2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;   //PA2 = UART2 TXD
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;   //PA3 = UART2 RXD
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART2, ENABLE);	

  //init UART1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;   //PA9 = UART1 TXD
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;   //PA10 = UART1 RXD
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);

  //PA8 to control RST pin of Lora board
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  //configure NVIC
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Print_Log(char *log)
{
  while(*log != NULL)
  {
    USART_SendData(USART2, *log);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);  
    log++;
  }
}
uint8_t Tx_Log[300];
int Log_Index;
void serialSendString(char *str)
{
  while(*str != NULL)
  {
    USART_SendData(USART1, *str);
    Tx_Log[Log_Index++] = *str;
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  
    str++;
  }
}

void serialSendBuffer(uint8_t *buf, uint8_t len)
{
  uint8_t index=0;
  while(len--)
  {
    Tx_Log[Log_Index++] = buf[index];
    USART_SendData(USART1, buf[index++]);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  
  }
}

void resetResponseBuffer() {
  Response_Len = 0;
  memset(AT_Response, 0, MAX_RESPONSE_SIZE);
}

void resetTxBuffer() {
  Tx_Len = 0;
  memset(Tx_Data, 0, MAX_TX_BUF_SIZE);
}

bool Send_And_Wait(char *at_cmd, char *expected)
{   
  Response_Len = 0;
  Is_Response_Avail = 0; 
  //send AT command
  while(*at_cmd != NULL)
  {
    USART_SendData(USART1, *at_cmd);
    Tx_Log[Log_Index++] = *at_cmd;
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);  
    at_cmd++;
  }
  //wait for response
  delay(300);

  if(!Is_Response_Avail)  return false;
  
  //check response
  if(memcmp(expected, AT_Response, Response_Len) == 0)
  {
    Print_Log("Lora Response:");
    Print_Log((char *)AT_Response);
    return true;
  }
  else
  {
    return false;
  }
}



void doReset(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_8);
  delay(10);
  GPIO_ResetBits(GPIOA, GPIO_Pin_8);
  delay(10);
  GPIO_SetBits(GPIOA, GPIO_Pin_8);
}

bool Lora_Receiving()
{
  Print_Log("Send AT+RECV\r\n");
  resetResponseBuffer();
  return Send_And_Wait("AT+RECV=zz\r\n", "OK\r\n");
}

void Lora_Sending(uint8_t *buf, uint8_t len)
{
  memset(Tx_Log, 0, 300);
  Log_Index = 0;
  len -= 2;   //to exlcude '\r\n' that is auto appended in Terminal program
  serialSendString("AT+SEND=");       //Send Tx command
  serialSendBuffer(&len, 1);            //Tx len
  serialSendBuffer(buf, len);           //Tx data
  serialSendString("\r\n");             //Ending characters
  delay(300);                           //wait for response
  if(Response_Len)
  {
    Print_Log("LoraResponse:");
    Print_Log((char *)AT_Response);
  }
}

int main(void)
{ 
  Board_Init();
  doReset();
  while(1)
  {
    switch(Control_State) 
    {
      case CHECKING_CONNECTION:
          Print_Log("Send AT\r\n");
          if(Send_And_Wait("AT\r\n", "OK\r\n"))
          {
            Control_State = SETTING_LORA;
          }
          break;
      case SETTING_LORA:
          //do lora setup
          Print_Log("Setting Lora....\r\n");

          //then move to Rx state
          Control_State = RECEIVING_DATA;
          Tx_Len = 0;
          Response_Len = 0;
          Lora_Receiving();
          Print_Log("Type any text to send:\r\n");
          break;
      case SENDING_AT_CMD:
          if(Is_Waiting_Response == 0) {
            Print_Log("Send AT cmd:");
            Print_Log((char *)Tx_Data);
            serialSendString((char *)Tx_Data);
            Is_Waiting_Response = 1;
          }
          if(Response_Len) {
            Is_Waiting_Response = 0;
            Print_Log("LoraResponse:");
            Print_Log((char *)AT_Response);
            Control_State = RECEIVING_DATA;
            resetTxBuffer();
            resetResponseBuffer();
            Print_Log("Type any text to send:\r\n");
          }
          break;
      case SENDING_DATA:
            Print_Log("Sending data: ");
            Print_Log((char *)Tx_Data);
            Lora_Sending(Tx_Data, Tx_Len);
            resetTxBuffer();
            resetResponseBuffer();
            Print_Log("Type any text to send:\r\n");
            Lora_Receiving();
            Control_State = RECEIVING_DATA;
          break;
      case RECEIVING_DATA:
          if(Tx_Len && Is_Receiving_Done) {
            if(memcmp(Tx_Data, "AT+", 3) == 0) {
              Control_State =  SENDING_AT_CMD;
            }
            else {
              Control_State = SENDING_DATA;
            }
          }
          if(Response_Len)
          {
            if(AT_Response[0] == '>') {
              Print_Log("Rx:");
              Print_Log(&AT_Response[1]);
            }
            else if (memcmp(AT_Response, "RX TIMEOUT\r\n", Response_Len) == 0) {
              Print_Log("LoraResponse:");
              Print_Log((char *)AT_Response);
              Lora_Receiving();
            } else {
              Print_Log("LoraResponse:");
              Print_Log((char *)AT_Response);
            }
            resetResponseBuffer();
            Print_Log("Type any text to send:\r\n");
          }
          break;
      default: break;
    }
    delay(300);
  }
}
