//
//  VoodooI2CPrecisionTouchpadHIDEventDriver.cpp
//  VoodooI2CHID
//
//  Created by Alexandre on 21/09/2017.
//  Copyright © 2017 Alexandre Daoud. All rights reserved.
//

#include "VoodooI2CPrecisionTouchpadHIDEventDriver.hpp"

#define super VoodooI2CMultitouchHIDEventDriver
OSDefineMetaClassAndStructors(VoodooI2CPrecisionTouchpadHIDEventDriver, VoodooI2CMultitouchHIDEventDriver);

void VoodooI2CPrecisionTouchpadHIDEventDriver::enterPrecisionTouchpadMode() {

    // retry limit
    if (!digitiser.input_mode) {

        uint64_t now;
        clock_get_uptime(&now);

        // soft reset window (1s)
        if (now - last_retry_time > 1000000000ULL) {
            retry_count = 0;
        }

        if (retry_count >= 10) {
            IOLog("VoodooI2CHID: retry limit reached\n");
            return;
        }

        retry_count++;
        last_retry_time = now;

        IOLog("VoodooI2CHID: input_mode not ready, retry %d\n", retry_count);

        if (!retry_scheduled) {

            retry_scheduled = true;

            uint64_t deadline;
            clock_interval_to_deadline(50, kMillisecondScale, &deadline);

            thread_call_enter_delayed(retry_call, deadline);
        }

        return;
    }

    // SUCCESS PATH
    retry_count = 0;
    retry_scheduled = false;

    UInt8 inputMode[] = { INPUT_MODE_TOUCHPAD };

    OSData *value = OSData::withBytes(inputMode, sizeof(inputMode));

    if (value) {
        digitiser.input_mode->setDataValue(value);
        OSSafeReleaseNULL(value);
        ready = true;
    }
}

void VoodooI2CPrecisionTouchpadHIDEventDriver::handleInterruptReport(
    AbsoluteTime timestamp,
    IOMemoryDescriptor *report,
    IOHIDReportType report_type,
    UInt32 report_id)
{
    if (!ready) {

        if (!retry_scheduled) {
            enterPrecisionTouchpadMode();
        }

        if (!ready)
            return;
    }

    super::handleInterruptReport(timestamp, report, report_type, report_id);
}

bool VoodooI2CPrecisionTouchpadHIDEventDriver::handleStart(IOService* provider) {

    if (!super::handleStart(provider))
        return false;

    if (!digitiser.input_mode)
        return false;

    retry_call = thread_call_allocate(
        [](thread_call_param_t param0, thread_call_param_t) {

            auto self =
                static_cast<VoodooI2CPrecisionTouchpadHIDEventDriver*>(param0);

            self->retry_scheduled = false;
            self->enterPrecisionTouchpadMode();

        },
        this
    );

    IOLog("%s::%s Putting device into Precision Touchpad Mode\n", getName(), name);

    ready = false;
    retry_count = 0;
    retry_scheduled = false;

    return true;
}

IOReturn VoodooI2CPrecisionTouchpadHIDEventDriver::parseElements(UInt32) {
    return super::parseElements(kHIDUsage_Dig_TouchPad);
}

IOReturn VoodooI2CPrecisionTouchpadHIDEventDriver::setPowerState(
    unsigned long whichState,
    IOService* whatDevice)
{
    if (whatDevice != this)
        return kIOReturnInvalid;

    if (!whichState) {
        ready = false;
    } else {
        if (!awake) {
            ready = false;
            retry_count = 0;
            retry_scheduled = false;
            awake = true;
        }
    }

    return kIOPMAckImplied;
}

void VoodooI2CPrecisionTouchpadHIDEventDriver::stop(IOService* provider) {

    if (retry_call) {
        thread_call_cancel(retry_call);
        thread_call_free(retry_call);
        retry_call = nullptr;
    }

    retry_scheduled = false;
    ready = false;

    super::stop(provider);
}
