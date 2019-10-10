#include "ros/ros.h"

#include "libs/lmic/src/lmic.h"
#include "libs/lmic/src/hal/hal.h"

#include "driving_node/move_side.h"
#include "compass_node/compass_raw.h"

#include "gps_node/gps_raw.h"
#include "compass_node/compass_raw.h"

#include "gps_nav_node/nav_to.h"
#include "gps_nav_node/turn_to.h"


#if !defined(DISABLE_INVERT_IQ_ON_RX)
#error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
       config.h in the lmic library to set it.
#endif

// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc
#define TX_INTERVAL 2000

// Dragino Raspberry PI hat (no onboard led)
// see https://github.com/dragino/Lora
#define RF_CS_PIN  RPI_V2_GPIO_P1_22 // Slave Select on GPIO25 so P1 connector pin #22
#define RF_IRQ_PIN RPI_V2_GPIO_P1_07 // IRQ on GPIO4 so P1 connector pin #7
#define RF_RST_PIN RPI_V2_GPIO_P1_11 // Reset on GPIO17 so P1 connector pin #11

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss  = RF_CS_PIN,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = RF_RST_PIN,
    .dio  = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
};

#ifndef RF_LED_PIN
#define RF_LED_PIN NOT_A_PIN
#endif

std::string data_last_tx;

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) {
}

osjob_t txjob;
osjob_t timeoutjob;

static void tx_func (osjob_t* job);



// Transmit the given string and call the given function afterwards
void tx(const char *str, osjobcb_t func) {
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
  LMIC.dataLen = 0;
  while (*str)
    LMIC.frame[LMIC.dataLen++] = *str++;
  LMIC.osjob.func = func;
  os_radio(RADIO_TX);
  // Serial.println("TX");
}
// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)
  os_radio(RADIO_RXON);
  // Serial.println("RX");
}

static void rxtimeout_func(osjob_t *job) {
  digitalWrite(LED_BUILTIN, LOW); // off
}


static void rx_func (osjob_t* job) {
  // Blink once to confirm reception and then keep the led on
  digitalWrite(LED_BUILTIN, LOW); // off
  delay(10);
  digitalWrite(LED_BUILTIN, HIGH); // on

  // Timeout RX (i.e. update led status) after 3 periods without RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);

  // Reschedule TX so that it should not collide with the other side's
  // next TX
  os_setTimedCallback(&txjob, os_getTime() + ms2osticks(TX_INTERVAL/2), tx_func);

  // Serial.print("Got ");
  // Serial.print(LMIC.dataLen);
  // Serial.println(" bytes");
  // Serial.write(LMIC.frame, LMIC.dataLen);
  // Serial.println();

  std::string data_tx(reinterpret_cast< char const* >(LMIC.frame));
  data_tx = data_tx.substr(0,LMIC.dataLen);

  if (data_tx != data_last_tx)
  {
    std::cout << data_tx << std::endl;
    std::vector<std::string> data_args;
    std::istringstream data_args_ss(data_tx);
    for(std::string v; data_args_ss >> v; )
        data_args.push_back(v);
    if(data_args[0] == "navto" && data_args.size() == 3)
    {
      float lat = std::stof(data_args[1].c_str());
      float lon = std::stof(data_args[2].c_str());
      ROS_INFO("nav_to called via lora");
      ROS_INFO("lat: %f", lat);
      ROS_INFO("lon: %f", lon);

  		gps_nav_node::nav_to navto;
  		navto.request.lat=lat;
      navto.request.lon=lon;

  		if (ros::service::call("nav_to", navto))
      {
        ROS_INFO("navto called");
      } else
      {
        ROS_INFO("navto call fail");
      }
    } else if(data_args[0] == "turnto" && data_args.size() == 2)
    {
      int dir = std::stoi(data_args[1].c_str());
      ROS_INFO("turnto called via lora");
      ROS_INFO("dir: %d", dir);

  		gps_nav_node::turn_to turnto;
  		turnto.request.dir=dir;

  		if (ros::service::call("turn_to", turnto))
      {
        ROS_INFO("turnto called");
      } else
      {
        ROS_INFO("turnto call fail");
      }
    }

  }


  data_last_tx = data_tx;
  // Restart RX
  rx(rx_func);
}

static void txdone_func (osjob_t* job) {
  rx(rx_func);
}

/*
  Local function to read gps location
*/
gps_node::gps_raw get_latest_gps_data()
{
  gps_node::gps_raw latest_gps_data = *ros::topic::waitForMessage<gps_node::gps_raw>("/gps_raw", ros::Duration(10));

  return latest_gps_data;
}

/*
  Local function to get compass dir
*/
int get_latest_dir()
{
  compass_node::compass_raw latest_dir = *ros::topic::waitForMessage<compass_node::compass_raw>("/compass_raw", ros::Duration(10));

  return latest_dir.dir;
}


// log text to USART and toggle LED
static void tx_func (osjob_t* job) {

  gps_node::gps_raw latest_gps_data = get_latest_gps_data();

  int live_heading = get_latest_dir();

  double live_lat = latest_gps_data.lat;
  double live_lon = latest_gps_data.lon;

  std::stringstream data;

  bool turn_to_srv_online;
  bool nav_to_srv_online;

  ros::param::get("/turning_to", turn_to_srv_online);
  ros::param::get("/naving_to", nav_to_srv_online);

  data << "dir:" << std::fixed << std::setprecision(6) << live_heading << ", lat:" << live_lat << ", lon:" << live_lon << ", turn_to:" << turn_to_srv_online << ", nav_to:" << nav_to_srv_online;

  ROS_INFO(data.str().c_str());

  tx(data.str().c_str(), txdone_func);
  // reschedule job every TX_INTERVAL (plus a bit of random to prevent
  // systematic collisions), unless packets are received, then rx_func
  // will reschedule at half this time.
  os_setTimedCallback(job, os_getTime() + ms2osticks(TX_INTERVAL + random(500)), tx_func);
}

// application entry point
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize runtime env
  os_init();

  // Set up these settings once, and use them for both TX and RX

#if defined(CFG_eu868)
  // Use a frequency in the g3 which allows 10% duty cycling.
  LMIC.freq = 869525000;
#elif defined(CFG_us915)
  LMIC.freq = 902300000;
#endif

  // Maximum TX power
  LMIC.txpow = 27;
  // Use a medium spread factor. This can be increased up to SF12 for
  // better range, but then the interval should be (significantly)
  // lowered to comply with duty cycle limits as well.
  LMIC.datarate = DR_SF9;
  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  os_setCallback(&txjob, tx_func);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lora com node");
  ros::NodeHandle n;

  // initing bcm lib, otherwise it will result in a segmentation fault
  if (!bcm2835_init())
    return 1;

  setup();

  while(ros::ok()) {
    // execute scheduled jobs and events
    os_runloop_once();

  }
  return 0;
}
