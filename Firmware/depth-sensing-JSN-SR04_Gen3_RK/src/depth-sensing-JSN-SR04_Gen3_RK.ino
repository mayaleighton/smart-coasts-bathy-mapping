#include "JSN-SR04_Gen3_RK.h"

const int TRIG_PIN = A2;
const int ECHO_PIN = A1;
const int UNUSED_PIN0 = A0;
const int UNUSED_PIN3 = A3;

SerialLogHandler logHandler;

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// Declare one of these as a global variable to manage the JSN-SR04 sensor
JSN_SR04_Gen3 distanceSensor;

void distanceCallback(JSN_SR04_Gen3::DistanceResult result) {
    switch(result.status) {
        case JSN_SR04_Gen3::DistanceResult::Status::SUCCESS:
            Log.info("cm=%lf inch=%lf", result.cm(), result.inch());
            break;

        case JSN_SR04_Gen3::DistanceResult::Status::RANGE_ERROR:
            Log.info("distance range error");
            break;

        default:
            Log.info("distance error %d", result.status);
            break;
    }
}

void setup() {

    // Initialize the sensor configuration from setup()
    distanceSensor
        .withTrigPin(TRIG_PIN)
        .withEchoPin(ECHO_PIN)
        .withUnusedPins(UNUSED_PIN0, UNUSED_PIN3)
        .withCallback(distanceCallback)
        .withSamplePeriodic(500ms)
        .setup();

}

void loop() {
    // You must call this frequently from loop(), preferable on every execution
    distanceSensor.loop();
}
