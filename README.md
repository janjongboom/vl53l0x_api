# vl530lx distance sensor - Mbed OS 5 library

Usage:

```cpp
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

VL53L0X_Dev_t distance_sensor;

VL53L0X_Error WaitMeasurementDataReady(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t NewDatReady=0;
    uint32_t LoopNb;

    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetMeasurementDataReady(Dev, &NewDatReady);
            if ((NewDatReady == 0x01) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return Status;
}

VL53L0X_Error WaitStopCompleted(VL53L0X_DEV Dev) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t StopCompleted=0;
    uint32_t LoopNb;

    if (Status == VL53L0X_ERROR_NONE) {
        LoopNb = 0;
        do {
            Status = VL53L0X_GetStopCompletedStatus(Dev, &StopCompleted);
            if ((StopCompleted == 0x00) || Status != VL53L0X_ERROR_NONE) {
                break;
            }
            LoopNb = LoopNb + 1;
            VL53L0X_PollingDelay(Dev);
        } while (LoopNb < VL53L0X_DEFAULT_MAX_LOOP);

        if (LoopNb >= VL53L0X_DEFAULT_MAX_LOOP) {
            Status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return Status;
}

void read_distance() {
    int measure = 0;
    int ave = 0;
    int sum = 0;

    VL53L0X_RangingMeasurementData_t RangingMeasurementData;

    // do 10 measurements
    for (size_t ix = 0; ix < 10; ix++) {
        WaitMeasurementDataReady(&distance_sensor);
        VL53L0X_GetRangingMeasurementData(&distance_sensor, &RangingMeasurementData);
        measure = RangingMeasurementData.RangeMilliMeter;
        sum += measure;

        // Clear the interrupt
        VL53L0X_ClearInterruptMask(&distance_sensor, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
        VL53L0X_PollingDelay(&distance_sensor);
    }

    // calculate the average of the 10 measurements
    ave = sum / 10;

    printf("Distance sensor: %d mm\n", ave);
}

int main() {
    EventQueue event_queue;
    Thread event_thread;
    event_thread.start(callback(&event_queue, &EventQueue::dispatch_forever));

    distance_sensor.I2cDevAddr      = 0x52;
    distance_sensor.comms_type      =  1;
    distance_sensor.comms_speed_khz =  400;

    VL53L0X_RdWord(&distance_sensor, VL53L0X_REG_OSC_CALIBRATE_VAL,0);
    VL53L0X_DataInit(&distance_sensor);
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    VL53L0X_StaticInit(&distance_sensor);
    VL53L0X_PerformRefSpadManagement(&distance_sensor, &refSpadCount, &isApertureSpads); // Device Initialization
    VL53L0X_PerformRefCalibration(&distance_sensor, &VhvSettings, &PhaseCal); // Device Initialization
    VL53L0X_SetDeviceMode(&distance_sensor, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode
    VL53L0X_SetLimitCheckValue(&distance_sensor, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536)); //High Accuracy mode, see API PDF
    VL53L0X_SetLimitCheckValue(&distance_sensor, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(18*65536)); //High Accuracy mode, see API PDF
    VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&distance_sensor, 200000); //High Accuracy mode, see API PDF
    VL53L0X_StartMeasurement(&distance_sensor);

    Ticker distance_ticker;
    distance_ticker.attach(event_queue.event(&read_distance), 5.0f);

    wait(osWaitForever);
}
```