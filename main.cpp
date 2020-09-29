#include "VL53L1X.h"
#include "mbed.h"

I2C *i2c;
VL53L1X *tof;
EventQueue *eventQueue;
DigitalOut led1(LED1);
int16_t offset;
uint16_t xtalk;

void blink() { led1 = !led1; }

void printDistance(uint16_t dist, uint8_t rangeStatus) {
  eventQueue->call(printf, "(%d) %d\r\n", rangeStatus, dist);
}

void calibrateTof() {
  int8_t result = 0;
  /*To benefit from the full performance of the sensor, the VL53L1X_ULD driver
    includes two calibration functions (offset and crosstalk) which need to be
    run once at the production line.  */
  result += tof->GetOffset(&offset);
  printf("old offset: %d\r\n", offset);
  result += tof->CalibrateOffset(100, &offset);
  printf("calibrated offset: %d\r\n", offset);
  result += tof->SetOffset(offset);
  result += tof->GetOffset(&offset);
  printf("new offset: %d\r\n", offset);
  result += tof->GetXtalk(&xtalk);
  printf("old xtalk: %d\r\n", xtalk);
  result += tof->CalibrateXtalk(100, &xtalk);
  printf("calibrated xtalk: %d\r\n", xtalk);
  result += tof->SetXtalk(xtalk);
  result += tof->GetXtalk(&xtalk);
  printf("new xtalk: %d\r\n", xtalk);
  printf("calibrate result: %d\r\n", result);
}

void initTof() {
  int8_t result = 0;
  uint8_t state;

  printf("[init tof]\r\n");
  result += tof->SensorReset();
  uint8_t timeout = 200;
  do {
    tof->BootState(&state);
    printf("bootstate: %d\r\n", state);
    wait_us(1000);
  } while (!state && timeout--);
  if (!timeout) {
    printf("#####TIMEOUT#####\r\n");
    return;
  }
  result += tof->ClearInterrupt();
  result += tof->SensorInit();

  //   calibrateTof();
  result += tof->SetOffset(25);
  result += tof->SetXtalk(0);

  printf("init result: %d\r\n", result);
}

#define PERIODE 50
void optionalTofFunctions() {
  int8_t result = 0;
  uint16_t interMeasurementInMs;
  VL53L1X::DistanceModes distanceMode;
  VL53L1X::TimingBudget timingBudgetInMs;
  printf("[optional tof functions]\r\n");
  result += tof->GetDistanceMode(&distanceMode);
  printf("default dist mode: %d\r\n");
  result += tof->SetDistanceMode(VL53L1X::DistanceModes::Long);
  result += tof->GetInterMeasurementInMs(&interMeasurementInMs);
  printf("default interMeasurement: %d\r\n", interMeasurementInMs);
  result += tof->SetInterMeasurementInMs(PERIODE); // default 100
  result += tof->GetTimingBudgetInMs(&timingBudgetInMs);
  printf("default TimingBudgetInMs: %d\r\n", timingBudgetInMs);
  result += tof->SetTimingBudgetInMs(
      VL53L1X::TimingBudget::_50ms); // 15 ms only works with Short
                                     // distance mode. 100 ms is the default
                                     // value
  result += tof->SetDistanceThreshold(100, 1000, VL53L1X::WindowTypes::Within);
  // result += tof->SetROICenter(uint8_t ROICenter);
  // result += tof->SetROI(uint16_t X, uint16_t Y);
  printf("optional functions result: %d\r\n", result);
}

void startRanging() {
  int8_t result = 0;
  printf("[start ranging]\r\n");
  tof->EnableInterrupt();
  tof->SetRangeStatusFilter(VL53L1X::RangeStatus::NoError);
  result += tof->StartRanging();
  printf("start ranging result: %d\r\n", result);
}

void interpretTofRangeStatus(VL53L1X::RangeStatus rangeStatus) {
  /*
0.. no error
[warnings]:
1.. sigma failure - This means that the repeatability or standard deviation of
the measurement is bad due to a decreasing signal noise ratio. Increasing
the timing budget can improve the standard deviation and avoid a range status
1
2..signal failure - there is a signal failure. This means that the return
signal is too week to return a good answer. The reason is because the target
is too far, or the target is not reflective enough, or the target is too
small. Increasing the timing buget might help, but there may simply be no
target available.
[errors]:
4.. out of bounds - This means that the sensor is ranging in
a “nonappropriated” zone and the measured result may be inconsistent. This
status is considered as a warning but, in general, it happens when a target is
at the maximum distance possible from the sensor, i.e. around 5 m. However,
this is only for very bright targets.
7.. wraparound - This situation may occur when the target is very reflective
and the distance to the target/sensor is longer than the physical limited
distance measurable by the sensor. Such distances include approximately 5 m
when the senor is in Long distance mode and approximately 1.3 m when the
sensor is in Short distance mode.
*/
  switch (rangeStatus) {
  case VL53L1X::RangeStatus::NoError:
    printf("no error\n\r");
    break;
  case VL53L1X::RangeStatus::SigmaFailure:
    printf("[ranging warning] sigma failure\n\r");
    break;
  case VL53L1X::RangeStatus::SignalFailure:
    printf("[ranging warning] signal failure\n\r");
    break;
  case VL53L1X::RangeStatus::RangingError:
    printf("[ranging error] out of bounds\n\r");
    break;
  case VL53L1X::RangeStatus::WrapAround:
    printf("[ranging error] wraparound\n\r");
    break;
  default:
    printf("unknown error\n\r");
    break;
  }
}

void getTofRanging() {
  int8_t result = 0;
  uint8_t dataReady, rangeStatus;
  uint16_t distance;
  //   printf("[get distance]\r\n");
  uint8_t timeout = 200;
  do {
    result += tof->CheckForDataReady(&dataReady);
    wait_us(1000);
  } while (!dataReady && timeout--);
  if (!timeout) {
    printf("#####TIMEOUT#####\r\n");
    return;
  }

  result += tof->GetRangeStatus(&rangeStatus);
  result += tof->GetDistance(&distance);
  if (rangeStatus) {
    interpretTofRangeStatus(static_cast<VL53L1X::RangeStatus>(rangeStatus));
  } else {
    printf("distance: %d mm\n\r", distance);
  }

  result += tof->ClearInterrupt();
  //   printf("ranging result: %d\r\n", result);
}

void getTofAmbient() {
  uint16_t amb, ambRate;
  tof->GetAmbientPerSpad(&amb);
  tof->GetAmbientRate(&ambRate);
  //   printf("amb: %d ambRate: %d\r\n", amb, ambRate);

  if (ambRate < 400) {
    led1 = 1;
  } else {
    led1 = 0;
  }
}

int main() {
  i2c = new I2C(I2C_SDA0, I2C_SCL0);
  tof = new VL53L1X(i2c, NC, p5);
  eventQueue = new EventQueue(32 * EVENTS_EVENT_SIZE);
  printf("Init\r\n");

  wait_us(1000);

  initTof();
  optionalTofFunctions();
  tof->SetInterruptCallback(callback(printDistance));
  startRanging();

  //   eventQueue->call_every(PERIODE, getTofRanging);
  //   eventQueue->call_every(500, blink);
  eventQueue->call_every(500, getTofAmbient);
  eventQueue->dispatch_forever();

  while (true) {
    sleep();
  }
  tof->StopRanging();
  return 0;
}