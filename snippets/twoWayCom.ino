// #######################################
const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
char commandd[numChars] = {0};

boolean newData = false;
boolean transmit = false;
// #######################################

void setup() {
  Serial2.begin(115200);
                  
  previous = millis();
  Serial.print("Hello bitches -- Initialization complete");
}

void loop() {

  if(millis()-previous >= SAMPLING_PERIOD*1000 && transmit){

    Serial2.println("  ");            
    previous = millis();
  }
  
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    showParsedData();
    newData = false;
   }
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial2.available() > 0 && newData == false) {
        rc = Serial2.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(tempChars,",");
    strcpy(commandd, strtokIndx); // copy it to messageFromPC
    
    if (strcmp(commandd, "start") == 0) {
      Serial2.println(commandd);
      transmit = true;
    }
    else if (strcmp(commandd, "change") == 0) {
      Serial2.println(commandd);
      transmit = false;
      strtokIndx = strtok(NULL,",");      
      K[0] = atof(strtokIndx);  
      strtokIndx = strtok(NULL, ","); 
      K[1] = atof(strtokIndx); 
      strtokIndx = strtok(NULL, ",");
      K[2] = atof(strtokIndx); 
    }
}

void showParsedData() {
    Serial2.print("f1 ");
    Serial2.println(K[0],4);
    Serial2.print("f2 ");
    Serial2.println(K[1],4);
    Serial2.print("f3 ");
    Serial2.println(K[2],4);
}