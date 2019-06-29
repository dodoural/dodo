/*
    SPI Slave Demo Sketch
    Connect the SPI Master device to the following pins on the esp8266:
    GPIO    NodeMCU   Name  |   Uno
  ===================================
     15       D8       SS   |   D10
     13       D7      MOSI  |   D11
     12       D6      MISO  |   D12
     14       D5      SCK   |   D13
    Note: If the ESP is booting at a moment when the SPI Master has the Select line HIGH (deselected)
    the ESP8266 WILL FAIL to boot!
    See SPISlave_SafeMaster example for possible workaround
*/


#include "SPISlave.h"
#include <string.h>
#include <ESP8266WiFi.h>
#include "hspi_slave.h"
#include "esp8266_peri.h"
#include "ets_sys.h"
#include <WiFiClientSecure.h> 
#include <ArduinoJson.h>
 extern "C" {
#include "user_interface.h"
}
#define WIFI_SSID     "ID"
#define WIFI_PASSWORD "PW"
#define TX_HASH       "TX"
#define REQUEST_NONCE "NC"
#define SSID_READY    (0b00000001)
#define PSWD_READY    (0b00000010)
#define TX_READY      (0b00000100)
#define CONNECTED     (0b00001000)
#define TX_BUSY       (0b00010000)
#define PREPARE_NONCE (0b00100000)
#define NONCE_READY   (0b01000000)
#define ADDRESS_SET   (0b10000000)


#define WIFI_READY (SSID_READY | PSWD_READY)
#define SET (0x00)
#define RESET (0x01)
#define MSG_SSID 0xFF
#define MSG_PSWD 0xFD
#define MSG_TX   0xFE
#define MSG_ADR  0xFA

void headerExtract(uint8_t* data,uint8_t* header );
void statusSet(uint8_t status, uint8_t state);
int wifiConnect();
uint32_t hexStringtoInt(String const &hex);

volatile uint8_t WLAN_SSID[128];
volatile uint8_t WLAN_PSWD[128];
uint8_t ETH_TX[1024];
uint8_t ADDRESS[128];
volatile uint32_t sts = 0;
volatile bool replaceShadowRegister = false;

//Web/Server address to read/write from 
const char *host = "testcore.evan.network";
const int httpsPort = 443;  //HTTPS= 443 and HTTP = 80

//SHA1 finger print of certificate use web browser to view and copy
const char fingerprint[] PROGMEM = "A8 C4 99 CD AC 6D 4E 55 AD EF 72 64 44 A2 F0 AA 44 FB 64 61";
char JSONmessageBuffer[1024];
char nonceMessageBuffer[256];

StaticJsonDocument<1800> JSONbuffer;
StaticJsonDocument<512> nonceBuffer;
StaticJsonDocument<256> parserBuffer;

WiFiClientSecure httpsClient;    //Declare object of class WiFiClient
JsonObject doc;
JsonObject nonceDoc;
JsonArray data;
JsonArray nonceData;
JsonObject parserObject;

void setup() {
 
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  WiFi.mode(WIFI_OFF);        //Prevents reconnection issue (taking too long to connect)
  delay(1000);
  WiFi.mode(WIFI_STA);        //Only Station No AP, This line hides the viewing of ESP as wifi hotspot
  pinMode(D4, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(D1, OUTPUT);
          
  nonceDoc= nonceBuffer.to<JsonObject>();
  nonceDoc["jsonrpc"] = "2.0";
  nonceDoc["method"] = "eth_getTransactionCount";
  nonceData = nonceDoc.createNestedArray("params");
  

 
  // data has been received from the master. Beware that len is always 32
  // and the buffer is autofilled with zeroes if data is less than 32 bytes long
  // It's up to the user to implement protocol for handling data length
  SPISlave.onData([](uint8_t * data, size_t len) {
    uint8_t header[3]={0};
    static bool firstCall = true;
    static short messageType = 0;
    static short messsageLength;
    static short messsageLength1;
    static short depth = 0;
    int recLen = (data[0]<<8)|data[1];
   /* for(int i = 0 ; i < 32; i++)
        {   
          Serial.printf("%c", data[i]);
        }
        Serial.println("");*/
        
    if(recLen <= 31 && firstCall == true)
    {
      headerExtract(data,header);
      
      if (strcmp((const char *)header,WIFI_SSID) == 0 )
      {
        
        memcpy((void*)WLAN_SSID,data+4,recLen-2);
        statusSet(SSID_READY,SET);
        SPISlave.setStatus(sts);
       /* for(int i = 0 ; i < strlen((const char *)WLAN_SSID); i++)
        {   
          Serial.printf("%c", WLAN_SSID[i]);
        }
        Serial.println("");*/

      }

      else if (strcmp((const char *)header,WIFI_PASSWORD) == 0 )
      {
        memcpy((void*)WLAN_PSWD,data+4,recLen-2);
        statusSet(PSWD_READY,SET);
        SPISlave.setStatus(sts);
      /*  for(int i = 0 ; i < strlen((const char *)WLAN_PSWD); i++)
        {   
          Serial.printf("%c", WLAN_PSWD[i]);
        }
        Serial.println("");*/

      }

      else if (strcmp((const char *)header,TX_HASH) == 0 )
      {
        memcpy((void*)ETH_TX,data+4,recLen-2);
        statusSet(TX_READY,SET);
        SPISlave.setStatus(sts);

      }
      else if (strcmp((const char *)header,REQUEST_NONCE) == 0 )
      {

        statusSet(PREPARE_NONCE,SET);
        SPISlave.setStatus(sts);

      }
          
    }
    
    else
    {
      if(firstCall)
      {

        firstCall = false;
        messsageLength = (data[0]<<8)|data[1];
        messsageLength1 = messsageLength;
        headerExtract(data,header);
     
        if (strcmp((const char *)header,WIFI_SSID) == 0 )
        {
          messageType = MSG_SSID;
          memset((void*)WLAN_SSID, 0 ,sizeof(WLAN_SSID));
          memcpy((void*)WLAN_SSID, data+4, 28);
         /* for(int i = 0 ; i < strlen((const char *)WLAN_SSID); i++)
          {   
            Serial.printf("%c", WLAN_SSID[i]);
          }
          Serial.println("");*/
        }
  
        else if (strcmp((const char *)header,WIFI_PASSWORD) == 0 )
        {
          messageType = MSG_PSWD;
          memset((void*)WLAN_PSWD, 0, sizeof(WLAN_PSWD));
          memcpy((void*)WLAN_PSWD, data+4, 28);         
    
        }
    
        else if (strcmp((const char *)header,TX_HASH) == 0 )
        {
          messageType = MSG_TX;
          memset((void*)ETH_TX, 0, sizeof(ETH_TX));
          memcpy((void*)ETH_TX, data+4, 28);
         /* Serial.println("FIRST MESSAGE");
          for(int i = 0 ; i < sizeof((const char *)ETH_TX); i++)
          {   
            Serial.printf("%d", ETH_TX[i]);
          }
          Serial.println("");*/
    
        }

        else if (strcmp((const char *)header,REQUEST_NONCE) == 0 )
        {
          messageType = MSG_ADR;
          memset((void*)ADDRESS, 0, sizeof(ADDRESS));
          memcpy((void*)ADDRESS, data+4, 28);         
        }
        
        depth = depth + 28 ;
        messsageLength = messsageLength - 30;
        
            
      }
      
      else
      {

        int tmpLength = 0;
        if(messsageLength > 32)
        {
          tmpLength = 32;
          messsageLength = messsageLength - 32;  
        }
        
        else
        {
          tmpLength = messsageLength;
          messsageLength = 0;
          firstCall = true;
        }
        
        if (messageType == MSG_SSID )
        {
          
          memcpy((void*)(WLAN_SSID + depth),data,tmpLength);
         
        }
  
        else if (messageType == MSG_PSWD )
        {
          memcpy((void*)(WLAN_PSWD + depth),data,tmpLength);
        
    
        }
    
        else if (messageType == MSG_TX )
        {
          memcpy((void*)(ETH_TX + depth),data,tmpLength);
          
        }

         else if (messageType == MSG_ADR )
        {
          memcpy((void*)(ADDRESS + depth),data,tmpLength);
          
        }


         if(!firstCall)
           depth = depth + 32 ;
           
         else
         {
            depth = 0;
            
            if (messageType == MSG_SSID )
            {
               statusSet(SSID_READY,SET);
               SPISlave.setStatus(sts);         
            }
  
            else if (messageType == MSG_PSWD )
            { 
              statusSet(PSWD_READY,SET);
              SPISlave.setStatus(sts);         
            }
      
            else if (messageType == MSG_TX )
            {
              statusSet(TX_READY,SET);          
              SPISlave.setStatus(sts);           
            } 
            
            else if (messageType == MSG_ADR )
            {
              statusSet(PREPARE_NONCE|ADDRESS_SET,SET);  
              nonceData.add((char*)ADDRESS);
              nonceData.add("latest");
              nonceDoc["id"] = 1;
              serializeJson(nonceDoc, nonceMessageBuffer);      
              SPISlave.setStatus(sts);           
            } 

         /*  for(int i = 0 ; i < messsageLength1; i++)
          {   
            Serial.printf("%d = %c",i, ETH_TX[i]);
                      Serial.println("");
          }
          Serial.println("");*/
         }
          

         
      }
   }

    
  });

  // The master has read out outgoing data buffer
  // that buffer can be set with SPISlave.setData
  SPISlave.onDataSent([]() {
    Serial.println("Answer Sent");
  });

  // status has been received from the master.
  // The status register is a special register that bot the slave and the master can write to and read from.
  // Can be used to exchange small data or status information
  SPISlave.onStatus([](uint32_t data) {
  });

  // The master has read the status register
  SPISlave.onStatusSent([]() {   
   // Serial.printf("%d STATUS sts\n",sts);         
  });

  // Setup SPI Slave registers and pins
  digitalWrite(D1,HIGH);    
  SPISlave.begin();
  // Sets the data registers. Limited to 32 bytes at a time.
  // SPISlave.setData(uint8_t * data, size_t len); is also available with the same limitation
 // SPISlave.setData("Ask me a question!");

}

void loop() {

noInterrupts();
wifiConnect();
sendEpochTime();
sendTx();
interrupts();

}



void statusSet(uint8_t status, uint8_t state)
{
  if(state == SET)
  sts = sts | status;
  else if (state == RESET)
  {
    sts = sts & (~status);
  }
}

int sendEpochTime()
{

if((sts & (CONNECTED|PREPARE_NONCE)) == (CONNECTED|PREPARE_NONCE))
{
    
    if(httpsClient.connected())
    {
      // Do nothing
    }
  
    else
    {
      int r=0; //retry counter
      while((!httpsClient.connect(host, httpsPort)) && (r < 30))
      {
        delay(100);
        r++;
      }
      
     if(r==30) 
     {
      Serial.println("Connection failed");
      if( WiFi.status() != WL_CONNECTED )
      {
        statusSet(CONNECTED,RESET);
      }
      SPISlave.setStatus(sts);

      return -1;
    }
    else 
    {
      Serial.println("Connected to web");
    }
    
    }
      
      httpsClient.print(String("POST ") + "/ HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: keep-alive\r\n"+
                 "Content-Type: application/json"+ "\r\n" +
                 "Content-Length: "+String(strlen(nonceMessageBuffer)) + "\r\n\r\n");
     
      httpsClient.print(nonceMessageBuffer); 
      while (httpsClient.connected()) {
        String line = httpsClient.readStringUntil('\n');
        if (line == "\r") {
          break;
        }
    }
      delay(100);
      String line;
      
      while(httpsClient.available())
      {  
         line = httpsClient.readStringUntil('\n');  //Read Line by Line 
         Serial.println(line);
         deserializeJson(parserBuffer, line);
         parserObject = parserBuffer.as<JsonObject>();
         String nonce = parserObject[String("result")];
         uint32_t a = hexStringtoInt(nonce);
         
         sts = sts & 0x000000FF;
         sts = sts | (a << 8);

      }
      statusSet(PREPARE_NONCE,RESET);
      statusSet(NONCE_READY,SET); 
      SPISlave.setStatus(sts);

   
  }
  
}
int wifiConnect()
{
  if( ( sts & (WIFI_READY|CONNECTED) ) ==  WIFI_READY)
  {
    if(WiFi.status() != WL_CONNECTED)   
    {
      size_t retry = 0;
      int a = strlen((const char *)WLAN_SSID);
      int b = strlen((const char *)WLAN_PSWD);
    
      WiFi.begin((const char *)WLAN_SSID, (const char *)WLAN_PSWD);
      
      while (WiFi.status() != WL_CONNECTED && retry < 20) 
      {
        retry++;
        delay(500);
        Serial.print(".");
      }
      delay(100);
      if(retry == 10)
      {
        
        statusSet(CONNECTED|WIFI_READY,RESET);
        SPISlave.setStatus(sts);
        Serial.print("CANT CONNECT WIFI");
        return -1;
      }
      statusSet(CONNECTED,SET);
      httpsClient.setFingerprint(fingerprint);
      httpsClient.setTimeout(3000); // 1,5 Seconds
      Serial.println("");
      Serial.print("Connected to ");
      Serial.println((const char *)WLAN_SSID);
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());  //IP address assigned to your ESP
    }

  }
  SPISlave.setStatus(sts);
  return 0;
  
}
uint32_t hexStringtoInt(String const &hex)
{
  uint32_t num = 0;
  size_t j = 0;
  uint32_t power = 0;
  size_t size = hex.length();  
  for(size_t i = size-1 ; i>=2 ; i--,j++)
  {
    if (j==0)
    {
      power = 1;  
    }
    
    else
    {
      power = 16<<((j-1)*4);  
    }
   
    if(hex[i] == 'a' || hex[i] == 'A')
    {
      num = num + 10*(power);
    }  

    else if(hex[i] == 'b' || hex[i] == 'B')
    {
      num = num + 11*(power);
    }  

    else if(hex[i] == 'c' || hex[i] == 'C')
    {
      num = num + 12*(power);
    }  

    else if(hex[i] == 'd' || hex[i] == 'D')
    {
      num = num + 13*(power);
    }  

    else if(hex[i] == 'e' || hex[i] == 'E')
    {
      num = num + 14*(power);
    }  
    
    else if(hex[i] == 'f' || hex[i] == 'F')
    {
      num = num + 15*(power);
    } 
    else
    {
      num = num + (hex[i]- 48)*power;
    }
   
  }  
   return num;
}
void sendTx()
{
  static bool ledVal = false;
  if((sts & (TX_READY|CONNECTED|NONCE_READY)) == (TX_READY|CONNECTED|NONCE_READY))
  {
    statusSet(TX_BUSY,SET);
    statusSet(TX_READY,RESET);
    SPISlave.setStatus(sts);
    if(httpsClient.connected())
    {
      // Do nothing
    }
  
    else
    {
      int r=0; //retry counter
      while((!httpsClient.connect(host, httpsPort)) && (r < 30))
      {
        delay(100);
        r++;
      }
      
     if(r==30) 
     {
      Serial.println("Connection failed");
      if( WiFi.status() != WL_CONNECTED )
      {
        statusSet(CONNECTED,RESET);
      }
      statusSet(TX_BUSY,RESET);
      SPISlave.setStatus(sts);

      return;
    }
    else 
    {
      Serial.println("Connected to web");
    }
    
    }
    
    doc = JSONbuffer.to<JsonObject>();
    doc["jsonrpc"] = "2.0";
    doc["method"] = "eth_sendRawTransaction";
    data = doc.createNestedArray("params");
    data.add((char*)ETH_TX);
    doc["id"] = 1;
        
    serializeJson(doc, JSONmessageBuffer);
    httpsClient.print(String("POST ") + "/post HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: keep-alive\r\n"+
                 "Content-Type: application/json"+ "\r\n" +
                 "Content-Length: "+String(strlen(JSONmessageBuffer)) + "\r\n\r\n");
     
      httpsClient.print(JSONmessageBuffer); 

      while (httpsClient.connected())
      {
        String line = httpsClient.readStringUntil('\n');
        if (line == "\r") 
        {
          break;
        }
      }
  
      delay(100);
      String line,httpReturn;
      
      while(httpsClient.available())
      {  
        line = httpsClient.readStringUntil('\n');  //Read Line by Line 
        Serial.println(line); //Print response
        if (memcmp(line.c_str(),"HTTP",4)==0)
        {  httpReturn = line.substring(9,12);
           Serial.println(httpReturn); //Print response
           ledVal = !ledVal;
           digitalWrite(D4,ledVal);   // Turn the LED on by making the voltage LOW
        }    
       
      } 
   
        statusSet(NONCE_READY,RESET);
        statusSet(TX_BUSY,RESET);
        SPISlave.setStatus(sts);
           ledVal = !ledVal;
           digitalWrite(D4,ledVal);   // Turn the LED on by making the voltage LOW
  }
}
  void headerExtract(uint8_t* data,uint8_t* header )
  {
    header[0] = data[2];
    header[1] = data[3];
    header[2] = 0;
  }
