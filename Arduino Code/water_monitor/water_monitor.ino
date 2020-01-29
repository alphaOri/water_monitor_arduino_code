/*
   Water monitoring/reporting arduino code written by David Smith
   Date: June 19, 2019
*/

#include <Ethernet.h>
#include <PubSubClient.h>
#include <EnableInterrupt.h>

//flow sensor related variables
//external interrupt can trigger from pins 2 or 3...
// using EnableInterrupt library for any other pins I need interrupts on
const byte flow_pulse_pin = 2, close_valve_indicator_pin = 9, open_valve_indicator_pin = 8,
          close_valve_cmd_pin = 7, open_valve_cmd_pin = 6, pressure_sensor_pwr_pin=5, pressure_analog_pin=A0;
const float liters_per_pulse = 0.00347222222; //from flow meter documentation
volatile int num_of_pulses = 0, pulses_this_period = 0;
volatile bool period_up = false;
volatile byte reports_per_second = 1;
volatile char valve_status_string[8];
volatile bool valve_status_message_present = false;
uint32_t last_interrupt_time = 0; //for debouncing

//pressure sensor variables
int pressure_reading = 0;

// related variables
byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
IPAddress server(192, 168, 42, 1);
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

#define DEBOUNCE_DELAY 100 // in ms
#define ARDUINO_CLIENT_ID "water_monitor"

#define PUB_FLOW_METER "water_monitor/flow_meter"
#define PUB_VALVE_STATUS "water_monitor/valve_status"
#define PUB_CONFIG_STATUS "water_monitor/config_status"
#define PUB_PRESSURE_METER "water_monitor/pressure"

#define SUB_VALVE_CONTROL "water_monitor/valve_control"
#define SUB_CONFIG "water_monitor/config"
#define SUB_STATUS_REQUEST "water_monitor/status_request"

void setup() {
  //Flow meter pins setup
  pinMode(flow_pulse_pin, INPUT); //water flow pulse pin is input for interrupt
  //ISR triggers on a low to high transition on flow pulse pin (0 is pin 2, 1 is pin 3)
  enableInterrupt(flow_pulse_pin, ISR_water_flow_pulse, RISING);

  //Valve pins setup
  pinMode(close_valve_indicator_pin, INPUT_PULLUP);
  pinMode(open_valve_indicator_pin, INPUT_PULLUP);
  enableInterrupt(close_valve_indicator_pin, ISR_valve_closed_indicator, CHANGE);
  enableInterrupt(open_valve_indicator_pin, ISR_valve_open_indicator, CHANGE);
  pinMode(close_valve_cmd_pin, OUTPUT);
  pinMode(open_valve_cmd_pin, OUTPUT);

  //Pressure pins setup - providing power to the pressure sensor via digital pin since ran out of board space...
  pinMode(pressure_sensor_pwr_pin, OUTPUT);
  digitalWrite(pressure_sensor_pwr_pin, HIGH);

  //Serial setup
  //Serial.begin(115200);

  setupTimer1(reports_per_second);

  //MQTT setup
  mqttClient.setServer(server, 1883);
  mqttClient.setCallback(MQTT_callback);

  //Ethernet setup
  //Serial.println("Initialize Ethernet with DHCP:");
  Ethernet.init(10);
  eth_connect();

  //need an init function which resends startup values incase of power cycle
}

void loop() {
  float flow_rate;
  
  if (!mqttClient.connected()) {
    mqtt_reconnect();
  }
  mqttClient.loop();
  Ethernet.maintain();

  if(valve_status_message_present) {
    //Serial.println("Sending valve status");
    mqttClient.publish(PUB_VALVE_STATUS, valve_status_string);
    valve_status_message_present = false;
  }

  if (period_up) {

    /*send flow rate*/
    //Flow rate (l/m) = calibration_factor * num_of_pulses / pulses_per_liter / time_period
    //flow_rate = calibration_factor * pulses_this_period * liters_per_pulse * reports_per_second;
    char flowRateBuffer[10];
    sprintf(flowRateBuffer, "%d", pulses_this_period);
    mqttClient.publish(PUB_FLOW_METER, flowRateBuffer);

    //Serial.print("pulses_this_period = ");
    //Serial.print(pulses_this_period);
    //Serial.print("\n");

    /*send pressure reading*/
    pressure_reading = analogRead(pressure_analog_pin);
    char pressureBuffer[10];
    sprintf(pressureBuffer, "%d", pressure_reading);
    mqttClient.publish(PUB_PRESSURE_METER, pressureBuffer);

    period_up = false; //reset, wait for Timer1 ISR to set again
  }
}

//triggered by low to high transition on pin from flow meter
void ISR_water_flow_pulse () {
  num_of_pulses++;
}

//triggered by logic change. input low only when valve is fully closed
void ISR_valve_closed_indicator () {
  uint32_t interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY) {
    //Serial.println(interrupt_time-last_interrupt_time);
    if (digitalRead(close_valve_indicator_pin)==LOW) {
      //Serial.println("valve is closed.");
      strcpy(valve_status_string, "closed");
    }
    else {
      //Serial.println("valve is opening.");
      strcpy(valve_status_string, "opening");
    }
    valve_status_message_present = true;
  }
  last_interrupt_time = interrupt_time;
}

//triggered by logic change. input low only when valve is fully open
void ISR_valve_open_indicator () {
  uint32_t interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > DEBOUNCE_DELAY) {
    //Serial.println(interrupt_time-last_interrupt_time);
    if (digitalRead(open_valve_indicator_pin)==LOW) {
      //Serial.println("valve is open");
      strcpy(valve_status_string, "open");
    }
    else {
      //Serial.println("valve is closing");
      strcpy(valve_status_string, "closing");
    }
    valve_status_message_present = true;
  }
  last_interrupt_time = interrupt_time;
}

//triggered by Timer1 output compare unit every set time period
ISR(TIMER1_COMPA_vect) {
  period_up = true;
  pulses_this_period = num_of_pulses;
  num_of_pulses = 0;
}

void setupTimer1(int per_second) {
  //Timer1 setup - trigger output compare interrupt every second
  TCCR1A = 0;  TCCR1B = 0; //need to disable timer1 before setting OCR1A or it won't take
  OCR1A = 62458/per_second; //16000000/256=62500, empircally adjusted to 62458
  TCCR1B = 0b00001100; //prescaler set to 256
  TIMSK1 = 0b00000010; //enable output compare A interrupt
}

void eth_connect(){
  while (Ethernet.begin(mac) == 0) {
    //Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      //Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    }
    delay(1000);
  }
  //Serial.print("  DHCP assigned IP ");
  //Serial.println(Ethernet.localIP());
}

void mqtt_reconnect()
{
  byte tries=0;
  // Loop until reconnected
  while (!mqttClient.connected()) {
    //Serial.print("Attempting MQTT connection ... ");
    // Attempt to connect
    if (mqttClient.connect(ARDUINO_CLIENT_ID)) {
      //Serial.println("connected");
      // (re)subscribe
      if (!mqttClient.subscribe(SUB_VALVE_CONTROL)) {
        //Serial.print("Failed: Subscribe to ");
        //Serial.println (SUB_VALVE_CONTROL);
      }
      else {
        //Serial.print("Successful: Subscribe to ");
        //Serial.println(SUB_VALVE_CONTROL);
      }
      if (!mqttClient.subscribe(SUB_CONFIG)) {
        //Serial.print("Failed: Subscribe to ");
        //Serial.println (SUB_CONFIG);
      }
      else {
        //Serial.print("Successful: Subscribe to ");
        //Serial.println(SUB_CONFIG);
      }
      if (!mqttClient.subscribe(SUB_STATUS_REQUEST)) {
        //Serial.print("Failed: Subscribe to ");
        //Serial.println (SUB_STATUS_REQUEST);
      }
      else {
        //Serial.print("Successful: Subscribe to ");
        //Serial.println(SUB_STATUS_REQUEST);
      }
    } else {
      //Serial.print("Connection failed, state: ");
      //Serial.print(mqttClient.state());
      //Serial.println(", retrying in 5 seconds");
      delay(5000); // Wait 5 seconds before retrying
      tries++;
      if(tries>24){ // every 2 minutes
        tries=0;
        ethClient.stop();
        eth_connect();
      }
    }
  }
}

void MQTT_callback(char* topic, byte* payload, unsigned int length) {
  //Serial.print("[sub: ");
  //Serial.print(topic);
  //Serial.print("] ");
  char message[length + 1] = "";
  for (int i = 0; i < length; i++)
    message[i] = (char)payload[i];
  message[length] = '\0';
  //Serial.println(message);

  if (strcmp(topic, SUB_VALVE_CONTROL)==0) {
    if (strcmp(message, "open")==0) {
      //Serial.println("opening valve.");
      digitalWrite(close_valve_cmd_pin, LOW);
      digitalWrite(open_valve_cmd_pin, HIGH);
    }
    else if (strcmp(message, "close")==0) {
      //Serial.println("closing valve.");
      digitalWrite(open_valve_cmd_pin, LOW);
      digitalWrite(close_valve_cmd_pin, HIGH);
    }
  }
  else if (strcmp(topic, SUB_CONFIG)==0) {
    //Serial.println("configuring");
    char *strings[3];
    char *ptr = NULL;
    byte index = 0;
    ptr = strtok(message, ":;");  // takes a list of delimiters
    //SUB_CONFIG format is reports_per_second:2
    while(ptr != NULL) {
        strings[index] = ptr;
        index++;
        ptr = strtok(NULL, ":;");  // takes a list of delimiters
    }
    if (strcmp(strings[0], "reports_per_second") == 0){
      reports_per_second = strtod(strings[1], NULL);
      //Serial.print("reports_per_second set to: ");
      //Serial.println(reports_per_second);
      // need to reset timer1 and num_of_pulses
      noInterrupts();
      setupTimer1(reports_per_second);
      num_of_pulses = 0;
      interrupts();
    }
    /*if (strcmp(strings[2], "calibration_factor") == 0){
      calibration_factor = strtod(strings[3], NULL);
      Serial.print("calibration_factor set to: ");
      Serial.println(calibration_factor);
    }*/
  }
  else if (strcmp(topic, SUB_STATUS_REQUEST)==0) {
    //Serial.println("startup status request received.");
    //return valve status
    if (digitalRead(close_valve_indicator_pin)==LOW) {
      //Serial.println("valve is closed.");
      mqttClient.publish(PUB_VALVE_STATUS, "closed");
    }
    else if (digitalRead(open_valve_indicator_pin)==LOW) {
      //Serial.println("valve is open");
      mqttClient.publish(PUB_VALVE_STATUS, "open");
    }
    else {
      //Serial.println("valve is ajar");
      mqttClient.publish(PUB_VALVE_STATUS, "ajar");
    }
    //return report frequency
    char config_message[25];
    char tmpBuffer[7];
    sprintf(config_message, "{\"reports_per_second\":%d}", reports_per_second);
    //Serial.print("reports_per_second: ");
    //Serial.println(reports_per_second);
    mqttClient.publish(PUB_CONFIG_STATUS, config_message);
  }

}
