#define DELAY_TIME 10 // unit: ms
#define BUFFER_SIZE 10

// Output Pin (LED 燈號)
#define GPS_TX_LED_OUTPIN 2
#define WARNING_RX_LED_OUTPIN 3
#define DIRECTION_LIGHT_LED_OUTPIN 4

// Input Pin
#define DIRECTION_LIGHT_CHECK_INPIN 5
#define START_SWITCH_INPIN 6
#define BUZZER_SWITCH_INPIN 7

// Output Pin
#define SVIDEO_HPR_OUTPIN 8
#define SVIDEO_TVOUT_OUTPIN 9
#define NH25_SIGNAL_OUTPIN 10
#define VCC24V_OUTPIN 11
#define VCC12V_OUTPIN 12

boolean initFlag = false;
unsigned char gpsCount = 0, checkFrequencyCount = 0, buzzerCount = 0;
int buzzerSwitchStatus = 0, startSwitchStatus = 0;

char recvBuffer[BUFFER_SIZE] = {0}; // RX LDW or FCW data
char warningMessage_1[7] = {'$', 'M', 'W', 'L', 'D', 'T', ','};   // $MWLDT,
char warningMessage_2[7] = {'$', 'M', 'W', 'F', 'C', 'W', ','};   // $MWFCW,
char warningMessage_3[5] = {'A', 'T', '$', 'P', 'D'};             // AT$PD=21;LDW:1<0x0D><0x0A>

unsigned char tempPositiveVal = 0;
unsigned char tempNegativeVal = 0;
unsigned char positiveVal = 0;
unsigned char negativeVal = 0;

void setup()
{
  setupInit();
}

void loop()
{
  /***** Initial Flow *****/
  initFlow(); // only run once 

  /***** Check buzzer switch and start switch on off *****/
  checkSwitchStatus();
  
  if (buzzerSwitchStatus == HIGH && startSwitchStatus == LOW) // buzzer off, start on
  {
    /***** Send GPS to DVR *****/
    sendGPSData(); // 1s

    /***** Receive RX data from UART1 *****/
    receiveWarning(); // 10 ms

    /****** Check direction light frequency ******/
    int sensorVal_1 = digitalRead(DIRECTION_LIGHT_CHECK_INPIN); 
    float lowValueOfFrequency = 0.6; // unit: Hz
    float highValueOfFrequency = 1.5; // unit: Hz
    checkFrequencyRange(lowValueOfFrequency, highValueOfFrequency, sensorVal_1); // 100 ms

    /***** S-Video *****/
    checkSVideoWire();

    /***** NH2.5-2P *****/
    checkNH25Wire();

    /***** Power 24v, 12v *****/
    checkPowerWire();
    
    delay(DELAY_TIME);
  }
  else if (buzzerSwitchStatus == LOW && startSwitchStatus == HIGH) // buzzer on, start off
  {
    digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
    
    delay(500);
  }
  else  // buzzer and start on, or buzzer and start off
  {
    delay(500);
  }
}

void setupInit()
{
  // Input Pin
  pinMode(BUZZER_SWITCH_INPIN, INPUT_PULLUP);
  pinMode(START_SWITCH_INPIN, INPUT_PULLUP);
  pinMode(DIRECTION_LIGHT_CHECK_INPIN, INPUT_PULLUP);

  // Output Pin
  pinMode(GPS_TX_LED_OUTPIN, OUTPUT);
  pinMode(WARNING_RX_LED_OUTPIN, OUTPUT);
  pinMode(DIRECTION_LIGHT_LED_OUTPIN, OUTPUT);

  // Output Pin
  pinMode(SVIDEO_HPR_OUTPIN, OUTPUT);
  pinMode(SVIDEO_TVOUT_OUTPIN, OUTPUT);
  pinMode(NH25_SIGNAL_OUTPIN, OUTPUT);
  pinMode(VCC24V_OUTPIN, OUTPUT);
  pinMode(VCC12V_OUTPIN, OUTPUT);
  
  
  Serial.begin(9600);   // USB Debug
  Serial1.begin(9600);  // UART Communication
}

void initFlow()
{
  if (initFlag == false)
  {
    digitalWrite(GPS_TX_LED_OUTPIN, LOW);
    digitalWrite(WARNING_RX_LED_OUTPIN, LOW);
    digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, LOW);

    digitalWrite(SVIDEO_HPR_OUTPIN, LOW);
    digitalWrite(SVIDEO_TVOUT_OUTPIN, LOW);
    digitalWrite(NH25_SIGNAL_OUTPIN, LOW);
    digitalWrite(VCC24V_OUTPIN, LOW);
    digitalWrite(VCC12V_OUTPIN, LOW);
    delay(200);
    
    for (int i = 0; i < 3 ; i++) 
    {
      digitalWrite(GPS_TX_LED_OUTPIN, HIGH);
      digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
      digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, HIGH);
      delay(200);
      digitalWrite(GPS_TX_LED_OUTPIN, LOW);
      digitalWrite(WARNING_RX_LED_OUTPIN, LOW);
      digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, LOW);
      delay(200);
    }

    buzzerSwitchStatus = digitalRead(BUZZER_SWITCH_INPIN);
    startSwitchStatus = digitalRead(START_SWITCH_INPIN);
    initFlag = true;
  }
}

void checkSwitchStatus()
{
  int newBuzzerSwitchStatus = digitalRead(BUZZER_SWITCH_INPIN);
  int newStartSwitchStatus = digitalRead(START_SWITCH_INPIN);
  if (newBuzzerSwitchStatus != buzzerSwitchStatus || newStartSwitchStatus != startSwitchStatus)
  {
    // Output Pin reset
    digitalWrite(GPS_TX_LED_OUTPIN, LOW);
    digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, LOW);
    digitalWrite(WARNING_RX_LED_OUTPIN, LOW);
    digitalWrite(SVIDEO_HPR_OUTPIN, LOW);
    digitalWrite(SVIDEO_TVOUT_OUTPIN, LOW);
    digitalWrite(NH25_SIGNAL_OUTPIN, LOW);
    digitalWrite(VCC24V_OUTPIN, LOW);
    digitalWrite(VCC12V_OUTPIN, LOW);

    // Count reset
    gpsCount = 0;
    checkFrequencyCount = 0;
    buzzerCount = 0;

    // Other reset
    tempPositiveVal = 0;
    tempNegativeVal = 0;
    positiveVal = 0;
    negativeVal = 0;

    // record new value
    buzzerSwitchStatus = newBuzzerSwitchStatus;
    startSwitchStatus = newStartSwitchStatus;
  }
}

void sendGPSData()
{
  // Send GPS Data (ones per second)
  gpsCount++;
  if (gpsCount >= 100) // 100 * DELAY_TIME = 1000 ms = 1 s
  {
    // GPS TX Data
    Serial.println("Send GPS Data:");
    Serial.println("----------------------------------------------------------------------");
    Serial.println("$GPRMC,160530.00,A,2404.04359,N,12025.99974,E,44.444,,150617,,,A*72");
    Serial1.println("$GPRMC,160530.00,A,2404.04359,N,12025.99974,E,44.444,,150617,,,A*72");
  
    Serial.println("$GPGGA,160530.00,2404.04359,N,12025.99974,E,1,06,2.28,-13.3,M,16.1,M,,*4E");
    Serial1.println("$GPGGA,160530.00,2404.04359,N,12025.99974,E,1,06,2.28,-13.3,M,16.1,M,,*4E");


    // {test +
//    // $GPGGA
//    Serial.println("$GPGGA,121252.000,3937.3032,N,11611.6046,E,1,05,2.0,45.9,M,-5.7,M,,0000*77 (1)");
//    Serial1.println("$GPGGA,121252.000,3937.3032,N,11611.6046,E,1,05,2.0,45.9,M,-5.7,M,,0000*77 (1)");
//
//    // $GPRMC
//    Serial.println("$GPRMC,121252.000,A,3958.3032,N,11629.6046,E,15.15,359.95,070306,,,A*54 (2)");
//    Serial1.println("$GPRMC,121252.000,A,3958.3032,N,11629.6046,E,15.15,359.95,070306,,,A*54 (2)");
//
//    // GPVTG
//    Serial.println("$GPVTG,199.62,T,,M,53.712,N,99.475,K,A*3C (3)");
//    Serial1.println("$GPVTG,199.62,T,,M,53.712,N,99.475,K,A*3C (3)");
//
//    // $GPGGA
//    Serial.println("$GPGGA,121253.000,3937.3090,N,11611.6057,E,1,06,1.2,44.6,M,-5.7,M,,0000*72 (4)");
//    Serial1.println("$GPGGA,121253.000,3937.3090,N,11611.6057,E,1,06,1.2,44.6,M,-5.7,M,,0000*72 (4)");
//
//    // $GPGSA
//    Serial.println("$GPGSA,A,3,01,07,08,11,09,16,27,30,22,28,,,1.11,0.66,0.89*0E (5)");
//    Serial1.println("$GPGSA,A,3,01,07,08,11,09,16,27,30,22,28,,,1.11,0.66,0.89*0E (5)");
//
//    // $GPGSV
//    Serial.println("$GPGSV,4,2,13,09,10,222,31,11,84,335,35,16,13,104,27,17,08,255,30*70 (6)");
//    Serial1.println("$GPGSV,4,2,13,09,10,222,31,11,84,335,35,16,13,104,27,17,08,255,30*70 (6)");
    // test -}
    
    Serial.println("----------------------------------------------------------------------");

    // GPS LED
    digitalWrite(GPS_TX_LED_OUTPIN, HIGH);
    gpsCount = 0;  
  } 
  else if (gpsCount >= 25) // 25 * DELAY_TIME = 250 ms = 0.25 s
  {
    // GPS LED
    digitalWrite(GPS_TX_LED_OUTPIN, LOW);
  }
}

void receiveWarning()
{
  // $MWLDT,+0*XX
  // $MWFCW,0,0
  // AT$PD=21;LDW:1<0x0D><0x0A>
  int count = 0;
  int str1Length = sizeof(warningMessage_1) / sizeof(char);
  int str2Length = sizeof(warningMessage_2) / sizeof(char);
  int str3Length = sizeof(warningMessage_3) / sizeof(char);
  
  buzzerCount++;
  if (buzzerCount > 50) // 50 * 10 = 500 ms
  {
    digitalWrite(WARNING_RX_LED_OUTPIN, LOW); 
  }
  
  if (Serial1.available() > 0)
  {
    recvBuffer[count] = Serial1.read();
    Serial.write(recvBuffer[count]);
    
    // new warning message+
    if (recvBuffer[count] == 'A') 
    {
      count++;
      
      while (Serial1.available() > 0 && count < str3Length)
      {
        recvBuffer[count] = Serial1.read();
        Serial.write(recvBuffer[count]);
        count++;
      }
      
      // AT$PD=21;LDW:1<0x0D><0x0A>
      count = 0;
      for (int i = 0; i < str3Length; i++)
      {
        if (recvBuffer[i] == warningMessage_3[i])
          count++;
      }

      if (count == str3Length) {
        digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
        Serial.println();
        Serial.println("Success: Get warning message");
        buzzerCount = 0;
      }
    }
    // new warning message-
    
    // old warning message+    
//    if (recvBuffer[count] == '$')
//    {
//      count++;
//
//      while (Serial1.available() > 0 && count < max(str1Length, str2Length))
//      {
//        recvBuffer[count] = Serial1.read();
//        Serial.write(recvBuffer[count]);
//        count++;
//      }
//
//      // $MWLDT,
//      count = 0;
//      for (int i = 0; i < str1Length; i++) 
//      {
//        if (recvBuffer[i] == warningMessage_1[i])
//          count++;
//      }
//      
//      if (count == str1Length) {
//        digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
//        buzzerCount = 0;
//      }
//
//      // $MWFCW,
//      count = 0;
//      for (int i = 0; i < str2Length; i++) 
//      {
//        if (recvBuffer[i] == warningMessage_2[i])
//          count++;
//      }
//      
//      if (count == str2Length) {
//        digitalWrite(WARNING_RX_LED_OUTPIN, HIGH);
//        buzzerCount = 0;
//      }
//    }
    // old warning message-
  }
}

void checkFrequencyRange(float lowValue, float highValue, int sensorVal)
{
  // 67 count ~ 167 count
  float lowerBound = ((1 / highValue) * 1000 / DELAY_TIME);
  float upperBound = ((1/ lowValue) * 1000 / DELAY_TIME);

  // Count
  if (sensorVal == LOW)
  {
    tempNegativeVal += 1;
    
    if (tempPositiveVal > 0)
    {
      positiveVal = tempPositiveVal;
      tempPositiveVal = 0;
    }
  }
  else
  {
    tempPositiveVal += 1;
    
    if (tempNegativeVal > 0) 
    {
      negativeVal = tempNegativeVal;
      tempNegativeVal = 0;
    }
  }

  // Clear data
  if (tempNegativeVal > upperBound) // > 167 count
  {
    tempNegativeVal = 0;
    negativeVal = 0;
  }
  if (tempPositiveVal > upperBound) // > 167 count
  {
    tempPositiveVal = 0;
    positiveVal = 0;
  }

  checkFrequencyCount++;
  if (checkFrequencyCount >= 10) // 100 ms 檢查一次
  {
    unsigned char sum = negativeVal + positiveVal;
    Serial.print("negativeVal = ");
    Serial.println(negativeVal);
    Serial.print("positiveVal = ");
    Serial.println(positiveVal);
    
    if ((sum >= lowerBound) && (sum <= upperBound))
    {
      Serial.println("Success: Get direction light");
      Serial.print('%');
      Serial.println('0');
//      Serial.println(sum, DEC);
      digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, HIGH);
    }
    else
    {
      Serial.println("Failed: Get direction light");
      Serial.print('%');
      Serial.println('1');
//      Serial.println(sum, DEC); 
      digitalWrite(DIRECTION_LIGHT_LED_OUTPIN, LOW);
    }

    checkFrequencyCount = 0;
  }
}

void checkSVideoWire()
{
  digitalWrite(SVIDEO_HPR_OUTPIN, HIGH);
  digitalWrite(SVIDEO_TVOUT_OUTPIN, HIGH);
}

void checkNH25Wire()
{
  digitalWrite(NH25_SIGNAL_OUTPIN, HIGH);
}

void checkPowerWire()
{
  digitalWrite(VCC24V_OUTPIN, HIGH);
  digitalWrite(VCC12V_OUTPIN, HIGH);
}

