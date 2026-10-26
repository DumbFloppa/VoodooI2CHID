//
//  VoodooI2CMultitouchHIDEventDriver.cpp
//  VoodooI2CHID
//
//  Created by Alexandre on 13/09/2017.
//  Copyright © 2017 Alexandre Daoud. All rights reserved.
//

#include "VoodooI2CMultitouchHIDEventDriver.hpp"
#include <IOKit/IOCommandGate.h>
#include <IOKit/hid/IOHIDInterface.h>
#include <IOKit/usb/USBSpec.h>
#include <IOKit/bluetooth/BluetoothAssignedNumbers.h>
#include <IOKit/IOLib.h>
#include <stdint.h>

#define super IOHIDEventService
OSDefineMetaClassAndStructors(VoodooI2CMultitouchHIDEventDriver, IOHIDEventService);

AbsoluteTime last_multi_touch_event = 0;

// Scale an unsigned HID value by base^exponent without wrapping on overflow.
// The physical range is ultimately stored in UInt32, so saturating is safer than
// allowing a malformed descriptor to wrap the value to a smaller dimension.
static UInt32 scientific_pow(UInt64 significand, UInt32 base, SInt32 exponent) {
    const UInt64 max_value = 0xFFFFFFFFULL;
    UInt64 result = significand;

    if (base == 0) {
        if (exponent < 0)
            return 0;
        return exponent == 0 ? (result > max_value ? (UInt32)max_value : (UInt32)result) : 0;
    }

    while (exponent > 0) {
        if (result > max_value / base)
            return (UInt32)max_value;
        result *= base;
        --exponent;
    }

    while (exponent < 0) {
        result /= base;
        ++exponent;
    }

    return result > max_value ? (UInt32)max_value : (UInt32)result;
}

void VoodooI2CMultitouchHIDEventDriver::calibrateJustifiedPreferredStateElement(IOHIDElement* element, SInt32 removal_percentage) {
    if (!element)
        return;

    if (removal_percentage < 0)
        removal_percentage = 0;

    // Removing 100% would collapse the calibrated interval to a single point.
    if (removal_percentage > 99)
        removal_percentage = 99;

    const SInt64 logical_min = element->getLogicalMin();
    const SInt64 logical_max = element->getLogicalMax();
    if (logical_max <= logical_min)
        return;

    const SInt64 range = logical_max - logical_min;
    const SInt64 diff = (range * removal_percentage) / 200;
    const SInt64 calibrated_min = logical_min + diff;
    const SInt64 calibrated_max = logical_max - diff;

    if (calibrated_min >= calibrated_max)
        return;

    element->setCalibration(0, 1, (SInt32)calibrated_min, (SInt32)calibrated_max);
}

bool VoodooI2CMultitouchHIDEventDriver::willTerminate(IOService* provider, IOOptionBits options) {
    if (hid_interface)
        hid_interface->close(this);
    hid_interface = NULL;

    return super::willTerminate(provider, options);
}

void VoodooI2CMultitouchHIDEventDriver::forwardReport(VoodooI2CMultitouchEvent event, AbsoluteTime timestamp) {
    if (multitouch_interface)
        multitouch_interface->handleInterruptReport(event, timestamp);
}

UInt32 VoodooI2CMultitouchHIDEventDriver::getElementValue(IOHIDElement* element) {
    if (!element || !hid_device)
        return 0;

    IOHIDElementCookie cookie = element->getCookie();
    if (!cookie)
        return 0;

    hid_device->updateElementValues(&cookie);
    return element->getValue();
}

const char* VoodooI2CMultitouchHIDEventDriver::getProductName() {
    VoodooI2CHIDDevice* i2c_hid_device = OSDynamicCast(VoodooI2CHIDDevice, hid_device);

    if (i2c_hid_device)
        return i2c_hid_device->name;

    if (OSString* name = getProduct())
        return name->getCStringNoCopy();

    return "Multitouch HID Device";
}

void VoodooI2CMultitouchHIDEventDriver::handleInterruptReport(AbsoluteTime timestamp, IOMemoryDescriptor* report, IOHIDReportType report_type, UInt32 report_id) {
    // Touchpad is disabled through ApplePS2Keyboard request
    if (ignore_all)
        return;

    uint64_t now_abs;
    clock_get_uptime(&now_abs);
    uint64_t now_ns;
    absolutetime_to_nanoseconds(now_abs, &now_ns);

    if (report_type == kIOHIDReportTypeInput && readyForReports())
        clock_get_uptime(&last_multi_touch_event);

    // Avoid unsigned underflow if the key timestamp is unexpectedly in the future.
    // Ignore touchpad interaction(s) shortly after typing.
    if (now_ns < key_time || now_ns - key_time < max_after_typing)
        return;

    if (!readyForReports() || report_type != kIOHIDReportTypeInput)
        return;

    if (digitiser.contact_count) {
        const UInt32 contact_count = digitiser.contact_count->getValue();
        digitiser.current_contact_count = contact_count;
        digitiser.current_report = 1;

        if (contact_count == 0) {
            // A zero-contact report must be forwarded too, otherwise consumers can
            // retain stale contacts after all fingers have been lifted.
            digitiser.report_count = 1;
        } else {
            const UInt32 finger_count = digitiser.fingers ? digitiser.fingers->getCount() : 0;
            // Some pen-only devices expose Contact Count without any finger collections.
            digitiser.report_count = finger_count
                ? (contact_count + finger_count - 1) / finger_count
                : 1;
        }
    }

    if (digitiser.report_count == 0) {
        digitiser.report_count = 1;
        digitiser.current_report = 1;
    }

    handleDigitizerReport(timestamp, report_id);

    if (digitiser.current_report == digitiser.report_count) {
        VoodooI2CMultitouchEvent event;
        event.contact_count = digitiser.current_contact_count;
        event.transducers = digitiser.transducers;

        forwardReport(event, timestamp);

        digitiser.report_count = 1;
        digitiser.current_report = 1;
    } else {
        digitiser.current_report++;
    }
}

void VoodooI2CMultitouchHIDEventDriver::handleDigitizerReport(AbsoluteTime timestamp, UInt32 report_id) {
    if (!digitiser.transducers || !digitiser.wrappers || !digitiser.fingers ||
        !digitiser.styluses || digitiser.current_report == 0)
        return;

    const UInt32 wrapper_index = digitiser.current_report - 1;
    if (wrapper_index >= digitiser.wrappers->getCount())
        return;

    VoodooI2CHIDTransducerWrapper* wrapper = OSDynamicCast(
        VoodooI2CHIDTransducerWrapper, digitiser.wrappers->getObject(wrapper_index));
    if (!wrapper || !wrapper->transducers)
        return;

    const UInt32 finger_count = digitiser.fingers->getCount();

    if (finger_count) {
        // Check if we are sending the report to the right wrapper, 99% of the time, `digitiser.current_report - 1` will
        // be the correct index but in rare circumstances, it won't be so we should ensure we have the right index

        if (wrapper->first_identifier) {
            const UInt32 first_identifier = wrapper->first_identifier->getValue();

            // Round up the result of division. Keep the calculation wide enough to
            // prevent UInt8 overflow when a device reports a large contact identifier.
            const UInt64 actual_index = ((UInt64)first_identifier + finger_count) / finger_count - 1;

            if (actual_index != wrapper_index && actual_index < digitiser.wrappers->getCount()) {
                VoodooI2CHIDTransducerWrapper* indexed_wrapper = OSDynamicCast(
                    VoodooI2CHIDTransducerWrapper, digitiser.wrappers->getObject((UInt32)actual_index));
                if (indexed_wrapper && indexed_wrapper->transducers)
                    wrapper = indexed_wrapper;
            }
        }
    }

    for (int i = 0; i < wrapper->transducers->getCount(); i++) {
        VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer, wrapper->transducers->getObject(i));
        if (transducer) {
            handleDigitizerTransducerReport(transducer, timestamp, report_id);
        }
    }

    // Now handle button report
    if (digitiser.primaryButton) { // there can't be secondary button without primary
        VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer, digitiser.transducers->getObject(0));
        if (transducer) {
            setButtonState(&transducer->physical_button, 0, digitiser.primaryButton->getValue(), timestamp);
            if (digitiser.secondaryButton) {
                transducer->has_secondary_button = true;
                setButtonState(&transducer->physical_button, 1, digitiser.secondaryButton->getValue(), timestamp);
            }
        }
    }

    if (digitiser.styluses->getCount() > 0) {
        // The stylus wrapper is the last one.
        wrapper = OSDynamicCast(VoodooI2CHIDTransducerWrapper, digitiser.wrappers->getLastObject());
        if (!wrapper || !wrapper->transducers || wrapper->transducers->getCount() == 0)
            return;

        VoodooI2CDigitiserStylus* stylus = OSDynamicCast(
            VoodooI2CDigitiserStylus, wrapper->transducers->getObject(0));
        if (!stylus || !stylus->collection)
            return;

        OSArray* stylus_elements = stylus->collection->getChildElements();
        if (!stylus_elements || stylus_elements->getCount() == 0)
            return;

        IOHIDElement* element = OSDynamicCast(IOHIDElement, stylus_elements->getObject(0));
        if (element && report_id == element->getReportID())
            handleDigitizerTransducerReport(stylus, timestamp, report_id);
    }
}

void VoodooI2CMultitouchHIDEventDriver::handleDigitizerTransducerReport(VoodooI2CDigitiserTransducer* transducer, AbsoluteTime timestamp, UInt32 report_id) {
    bool has_confidence = false;
    bool has_valid = false;
    UInt32 element_index = 0;
    UInt32 element_count = 0;

    if (!transducer || !transducer->collection)
        return;

    OSArray* child_elements = transducer->collection->getChildElements();

    if (!child_elements)
        return;

    for (element_index=0, element_count=child_elements->getCount(); element_index < element_count; element_index++) {
        IOHIDElement* element;
        AbsoluteTime element_timestamp;
        UInt32 usage_page;
        UInt32 usage;
        UInt32 value;

        // Only stylus transducers carry the additional pen-specific state below.
        VoodooI2CDigitiserStylus* stylus = OSDynamicCast(VoodooI2CDigitiserStylus, transducer);

        element = OSDynamicCast(IOHIDElement, child_elements->getObject(element_index));
        if (!element)
            continue;

        element_timestamp = element->getTimeStamp();

        transducer->id = report_id;
        transducer->timestamp = element_timestamp;

        usage_page = element->getUsagePage();
        usage = element->getUsage();
        value = element->getValue();

        switch (usage_page) {
            case kHIDPage_GenericDesktop:
                switch (usage) {
                    case kHIDUsage_GD_X:
                    {
                        transducer->coordinates.x.update(element->getValue(), timestamp);
                        transducer->logical_max_x = element->getLogicalMax();
                        break;
                    }
                    case kHIDUsage_GD_Y:
                    {
                        transducer->coordinates.y.update(element->getValue(), timestamp);
                        transducer->logical_max_y = element->getLogicalMax();
                        break;
                    }
                    case kHIDUsage_GD_Z:
                    {
                        transducer->coordinates.z.update(element->getValue(), timestamp);
                        transducer->logical_max_z = element->getLogicalMax();
                        break;
                    }
                }
                break;
            case kHIDPage_Button:
                if (usage == kHIDUsage_Button_1) {
                    // kHIDUsage_Button_1 is for the clickpad surface.
                    setButtonState(&transducer->physical_button, 0, value, timestamp);
                } else if (usage >= kHIDUsage_Button_2) {
                    // kHIDUsage_Button_2/3 are for external buttons, thus should be
                    // reported through the trackpoint device.
                    transducer->has_secondary_button = true;
                    setButtonState(&transducer->physical_button, (usage - 2), value, timestamp);
                }
                break;
            case kHIDPage_Digitizer:
                switch (usage) {
                    case kHIDUsage_Dig_TransducerIndex:
                    case kHIDUsage_Dig_ContactIdentifier:
                        transducer->secondary_id = value;
                        break;
                    case kHIDUsage_Dig_Touch:
                    case kHIDUsage_Dig_TipSwitch:
                        setButtonState(&transducer->tip_switch, 0, value, timestamp);
                        break;
                    case kHIDUsage_Dig_InRange:
                        transducer->in_range = value != 0;
                        break;
                    case kHIDUsage_Dig_Confidence:
                    case kHIDUsage_Dig_Quality:
                        transducer->confidence.update(element->getValue(), timestamp);
                        has_confidence = true;
                        break;
                    case kHIDUsage_Dig_TipPressure:
                    case kHIDUsage_Dig_SecondaryTipSwitch:
                    {
                        transducer->tip_pressure.update(element->getValue(), timestamp);
                        transducer->pressure_physical_max = element->getPhysicalMax();
                        break;
                    }
                    case kHIDUsage_Dig_XTilt:
                        transducer->tilt_orientation.x_tilt.update(element->getScaledFixedValue(kIOHIDValueScaleTypePhysical), timestamp);
                        break;
                    case kHIDUsage_Dig_YTilt:
                        transducer->tilt_orientation.y_tilt.update(element->getScaledFixedValue(kIOHIDValueScaleTypePhysical), timestamp);
                        break;
                    case kHIDUsage_Dig_Azimuth:
                        transducer->azi_alti_orientation.azimuth.update(element->getValue(), timestamp);
                        break;
                    case kHIDUsage_Dig_Altitude:
                        transducer->azi_alti_orientation.altitude.update(element->getValue(), timestamp);
                        break;
                    case kHIDUsage_Dig_Twist:
                        transducer->azi_alti_orientation.twist.update(element->getScaledFixedValue(kIOHIDValueScaleTypePhysical), timestamp);
                        break;
                    case kHIDUsage_Dig_Width:
                        transducer->dimensions.width.update(element->getValue(), timestamp);
                        break;
                    case kHIDUsage_Dig_Height:
                        transducer->dimensions.height.update(element->getValue(), timestamp);
                        break;
                    case kHIDUsage_Dig_DataValid:
                        if (value)
                            transducer->is_valid = true;
                        else
                            transducer->is_valid = false;
                        has_valid = true;
                        break;
                    case kHIDUsage_Dig_BarrelPressure:
                        if (stylus) {
                            stylus->barrel_pressure.update(element->getScaledFixedValue(kIOHIDValueScaleTypeCalibrated), timestamp);
                        }
                        break;
                    case kHIDUsage_Dig_BarrelSwitch:
                        if (stylus) {
                            setButtonState(&stylus->barrel_switch, 1, value, timestamp);
                        }
                        break;
                    case kHIDUsage_Dig_BatteryStrength:
                        if (stylus) {
                            stylus->battery_strength = element->getValue();
                        }
                        break;
                    case kHIDUsage_Dig_Eraser:
                        if (stylus) {
                            setButtonState(&stylus->eraser, 2, value, timestamp);
                            stylus->invert = value != 0;
                        }
                        break;
                    case kHIDUsage_Dig_Invert:
                        if (stylus) {
                            stylus->invert = value != 0;
                        }
                        break;
                    default:
                        break;
                }
                break;
        }
    }

    if (!has_confidence)
        transducer->confidence.update(1, timestamp);

    if (!has_valid)
        transducer->is_valid = true;

}

bool VoodooI2CMultitouchHIDEventDriver::handleStart(IOService* provider) {
    if(!super::handleStart(provider)) {
        return false;
    }

    hid_interface = OSDynamicCast(IOHIDInterface, provider);

    if (!hid_interface)
        return false;

    OSString* transport = hid_interface->getTransport();
    if (!transport)
        return false;

    if (strncmp(transport->getCStringNoCopy(), kIOHIDTransportUSBValue, sizeof(kIOHIDTransportUSBValue)) != 0)

        hid_interface->setProperty("VoodooI2CServices Supported", kOSBooleanTrue);

    hid_device = OSDynamicCast(IOHIDDevice, hid_interface->getParentEntry(gIOServicePlane));

    if (!hid_device)
        return false;

    name = getProductName();

    OSObject* object = copyProperty(kIOHIDAbsoluteAxisBoundsRemovalPercentage, gIOServicePlane);

    OSNumber* number = OSDynamicCast(OSNumber, object);

    if (number) {
        absolute_axis_removal_percentage = number->unsigned32BitValue();
    }

    OSSafeReleaseNULL(object);

    if (should_have_interface && publishMultitouchInterface() != kIOReturnSuccess)
        return false;

    digitiser.fingers = OSArray::withCapacity(1);

    if (!digitiser.fingers)
        return false;

    digitiser.styluses = OSArray::withCapacity(1);

    if (!digitiser.styluses)
        return false;

    digitiser.transducers = OSArray::withCapacity(1);

    if (!digitiser.transducers)
        return false;

    if (parseElements(kHIDUsage_Dig_Any) != kIOReturnSuccess) {
        IOLog("%s::%s Could not parse multitouch elements\n", getName(), name);
        return false;
    }

    if (!hid_interface->open(this, 0, OSMemberFunctionCast(IOHIDInterface::InterruptReportAction, this, &VoodooI2CMultitouchHIDEventDriver::handleInterruptReport), NULL))
        return false;

    setDigitizerProperties();

    PMinit();
    hid_interface->joinPMtree(this);
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);

    return true;
}

void VoodooI2CMultitouchHIDEventDriver::handleStop(IOService* provider) {
    unregisterHIDPointerNotifications();
    OSSafeReleaseNULL(attached_hid_pointer_devices);

    if (multitouch_interface) {
        multitouch_interface->stop(this);
        multitouch_interface->detach(this);
        OSSafeReleaseNULL(multitouch_interface);
    }

    if (command_gate) {
        if (work_loop)
            work_loop->removeEventSource(command_gate);
        OSSafeReleaseNULL(command_gate);
    }

    OSSafeReleaseNULL(work_loop);


    OSSafeReleaseNULL(digitiser.transducers);
    OSSafeReleaseNULL(digitiser.wrappers);
    OSSafeReleaseNULL(digitiser.styluses);
    OSSafeReleaseNULL(digitiser.fingers);

    PMstop();
    super::handleStop(provider);
}

UInt32 VoodooI2CMultitouchHIDEventDriver::parseElementPhysicalMax(IOHIDElement* element) {
    if (!element)
        return 0;

    UInt64 physical_max = element->getPhysicalMax();

    // HID Unit Exponent is a signed 4-bit integer. Sign-extend it explicitly
    // rather than relying on pointer reinterpretation/aliasing.
    const UInt8 raw_unit_exponent = element->getUnitExponent() & 0x0F;
    SInt8 unit_exponent = (raw_unit_exponent & 0x08)
        ? (SInt8)(raw_unit_exponent | 0xF0)
        : (SInt8)raw_unit_exponent;

    // Scale to 0.01 mm units.
    const UInt32 unit = element->getUnit();
    if (unit == kHIDUsage_LengthUnitCentimeter) {
        unit_exponent += 3;
    } else if (unit == kHIDUsage_LengthUnitInch) {
        physical_max *= 254ULL;
        unit_exponent += 1;
    }

    return scientific_pow(physical_max, 10, unit_exponent);
}

IOReturn VoodooI2CMultitouchHIDEventDriver::parseDigitizerElement(IOHIDElement* digitiser_element) {
    if (!digitiser_element)
        return kIOReturnBadArgument;

    OSArray* children = digitiser_element->getChildElements();

    if (!children)
        return kIOReturnNotFound;

    // Let's parse the configuration page first

    if (digitiser_element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_DeviceConfiguration)) {
        for (int i = 0; i < children->getCount(); i++) {
            IOHIDElement* element = OSDynamicCast(IOHIDElement, children->getObject(i));
            if (!element)
                continue;

            if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_Finger)) {
                OSArray* sub_array = element->getChildElements();
                if (!sub_array)
                    continue;

                for (int j = 0; j < sub_array->getCount(); j++) {
                    IOHIDElement* sub_element = OSDynamicCast(IOHIDElement, sub_array->getObject(j));
                    if (sub_element && sub_element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_DeviceMode))
                        digitiser.input_mode = sub_element;
                }
            }
        }

        return kIOReturnSuccess;
    }

    for (int i = 0; i < children->getCount(); i++) {
        IOHIDElement* element = OSDynamicCast(IOHIDElement, children->getObject(i));
        if (!element)
            continue;

        if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_Stylus)) {
            if (!digitiser.styluses->setObject(element))
                return kIOReturnNoResources;
            setProperty("SupportsInk", 1, 32);
            continue;
        }

        if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_Finger)) {
            if (!digitiser.fingers->setObject(element))
                return kIOReturnNoResources;

            // Let's grab the logical and physical min/max while we are here
            // also the contact identifier

            OSArray* sub_array = element->getChildElements();
            if (!sub_array)
                continue;

            for (int j = 0; j < sub_array->getCount(); j++) {
                IOHIDElement* sub_element = OSDynamicCast(IOHIDElement, sub_array->getObject(j));
                if (!sub_element)
                    continue;

                if (sub_element->conformsTo(kHIDPage_GenericDesktop, kHIDUsage_GD_X)) {
                    if (multitouch_interface && !multitouch_interface->logical_max_x) {
                        multitouch_interface->logical_max_x = sub_element->getLogicalMax();
                        multitouch_interface->physical_max_x = parseElementPhysicalMax(sub_element);
                    }
                } else if (sub_element->conformsTo(kHIDPage_GenericDesktop, kHIDUsage_GD_Y)) {
                    if (multitouch_interface && !multitouch_interface->logical_max_y) {
                        multitouch_interface->logical_max_y = sub_element->getLogicalMax();
                        multitouch_interface->physical_max_y = parseElementPhysicalMax(sub_element);
                    }
                }
            }

            continue;
        }

        if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_ContactCount)) {
            digitiser.contact_count = element;
            continue;
        }

        if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_DeviceMode)) {
            digitiser.input_mode = element;
            continue;
        }

        if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_ContactCountMaximum)) {
            digitiser.contact_count_maximum = element;
            continue;
        }

        // On some machines (Namely Dell Latitude 7390 2-in-1) the primary button has kHIDUsage_Button_2, and the secondary button has kHIDUsage_Button_3
        // Address cases involving kHIDUsage_Button_3
        if (element->conformsTo(kHIDPage_Button) && element->getUsage() <= kHIDUsage_Button_3) {
            if (digitiser.primaryButton == nullptr) {
                digitiser.primaryButton = element;
            }
            else if (element->getUsage() > digitiser.primaryButton->getUsage()) {
                // Candidate for a secondary button
                if (digitiser.secondaryButton == nullptr || element->getUsage() < digitiser.secondaryButton->getUsage()) {
                    digitiser.secondaryButton = element;
                }
            }
            else if (element->getUsage() < digitiser.primaryButton->getUsage()) {
                // This is the new primary button. Old primary becomes secondary.
                digitiser.secondaryButton = digitiser.primaryButton;
                digitiser.primaryButton = element;
            }
        }
    }

    return kIOReturnSuccess;
}

IOReturn VoodooI2CMultitouchHIDEventDriver::parseElements(UInt32 usage) {
    int index, count;

    if (!hid_device || !digitiser.fingers || !digitiser.styluses || !digitiser.transducers)
        return kIOReturnNotReady;

    OSArray* supported_elements = OSDynamicCast(OSArray, hid_device->getProperty(kIOHIDElementKey));
    if (!supported_elements)
        return kIOReturnNotFound;

    for (index=0, count = supported_elements->getCount(); index < count; index++) {
        IOHIDElement* element = NULL;

        element = OSDynamicCast(IOHIDElement, supported_elements->getObject(index));
        if (!element)
            continue;

        if (element->getUsage() == 0)
            continue;

        /*
         * Parse digitzer elements depending on which Event Driver is attaching.
         * Precision/TouchScreen are specifically looking for kHIDUsage_Dig_TouchPad/TouchScreen respectively.
         * Generic Multitouch looks for any supported Digitizer element (usage == kHIDUsage_Dig_Any).
         * kHIDUsage_Dig_Pen and kHIDUsage_Dig_DeviceConfiguration are valid for both of the cases above.
         */

        if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_Pen)
            || element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_DeviceConfiguration)) {
            parseDigitizerElement(element);
        } else if (usage != kHIDUsage_Dig_Any && element->conformsTo(kHIDPage_Digitizer, usage)) {
            parseDigitizerElement(element);
        } else if (usage == kHIDUsage_Dig_Any
                   && (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_TouchScreen)
                       || element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_TouchPad))) {
            parseDigitizerElement(element);
        }

        if ((usage == kHIDUsage_Dig_Any || usage == kHIDUsage_Dig_TouchScreen)
            && multitouch_interface && element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_TouchScreen)) {
            multitouch_interface->setProperty(kIOHIDDisplayIntegratedKey, kOSBooleanTrue);
        }
    }


    if (digitiser.styluses->getCount() == 0 && digitiser.fingers->getCount() == 0)
        return kIOReturnError;

    digitiser.wrappers = OSArray::withCapacity(1);
    if (!digitiser.wrappers)
        return kIOReturnNoResources;

    // Pen-only devices may expose Contact Count Maximum without any finger
    // collections. Do not divide by zero or attempt to build finger wrappers.
    const UInt32 finger_collection_count = digitiser.fingers->getCount();
    if (digitiser.contact_count_maximum && finger_collection_count > 0) {
        const UInt32 contact_count_maximum = getElementValue(digitiser.contact_count_maximum);

        // Check if maximum contact count divides by digitiser finger count.
        if (contact_count_maximum % digitiser.fingers->getCount() != 0) {
            IOLog("%s::%s Unknown digitiser type: got %d finger collections and a %d maximum contact count, ignoring extra fingers\n",
                  getName(), name, digitiser.fingers->getCount(), contact_count_maximum);

            // Remove extra finger collections if the device advertises fewer
            // simultaneous contacts than collections in its descriptor.
            if (contact_count_maximum < digitiser.fingers->getCount()) {
                while (digitiser.fingers->getCount() > 0 &&
                       contact_count_maximum % digitiser.fingers->getCount() != 0) {
                    digitiser.fingers->removeObject(digitiser.fingers->getCount() - 1);
                }
            }
        }

        const UInt32 adjusted_finger_count = digitiser.fingers->getCount();
        const UInt32 wrapper_count = adjusted_finger_count
            ? contact_count_maximum / adjusted_finger_count : 0;

        for (UInt32 i = 0; i < wrapper_count; ++i) {
            VoodooI2CHIDTransducerWrapper* wrapper = VoodooI2CHIDTransducerWrapper::wrapper();
            if (!wrapper || !wrapper->transducers) {
                OSSafeReleaseNULL(wrapper);
                return kIOReturnNoResources;
            }
            if (!digitiser.wrappers->setObject(wrapper)) {
                IOLog("%s::%s Failed to add Transducer Wrapper to transducer array\n", getName(), name);
                OSSafeReleaseNULL(wrapper);
                return kIOReturnNoResources;
            }

            for (UInt32 j = 0; j < adjusted_finger_count; ++j) {
                IOHIDElement* finger = OSDynamicCast(IOHIDElement, digitiser.fingers->getObject(j));
                if (!finger) {
                    wrapper->release();
                    return kIOReturnError;
                }

                VoodooI2CDigitiserTransducer* transducer =
                    VoodooI2CDigitiserTransducer::transducer(kDigitiserTransducerFinger, finger);
                if (!transducer) {
                    wrapper->release();
                    return kIOReturnNoResources;
                }

                const bool added_to_wrapper = wrapper->transducers->setObject(transducer);
                const bool added_to_all = added_to_wrapper && digitiser.transducers->setObject(transducer);
                transducer->release();
                if (!added_to_all) {
                    wrapper->release();
                    return kIOReturnNoResources;
                }
            }

            VoodooI2CDigitiserTransducer* first_transducer = OSDynamicCast(
                VoodooI2CDigitiserTransducer, wrapper->transducers->getObject(0));
            if (first_transducer && first_transducer->collection) {
                OSArray* first_children = first_transducer->collection->getChildElements();
                if (first_children) {
                    for (UInt32 j = 0; j < first_children->getCount(); ++j) {
                        IOHIDElement* element = OSDynamicCast(IOHIDElement, first_children->getObject(j));
                        if (element && element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_ContactIdentifier))
                            wrapper->first_identifier = element;
                    }
                }
            }

            wrapper->release();
        }
    }

    // Add stylus as the final wrapper

    if (digitiser.styluses->getCount()) {
        VoodooI2CHIDTransducerWrapper* stylus_wrapper = VoodooI2CHIDTransducerWrapper::wrapper();
        if (!stylus_wrapper || !stylus_wrapper->transducers) {
            OSSafeReleaseNULL(stylus_wrapper);
            return kIOReturnNoResources;
        }

        IOHIDElement* stylus = OSDynamicCast(IOHIDElement, digitiser.styluses->getObject(0));
        if (!stylus) {
            stylus_wrapper->release();
            return kIOReturnError;
        }

        VoodooI2CDigitiserStylus* transducer =
            VoodooI2CDigitiserStylus::stylus(kDigitiserTransducerStylus, stylus);
        if (!transducer) {
            stylus_wrapper->release();
            return kIOReturnNoResources;
        }

        const bool added_to_wrapper = stylus_wrapper->transducers->setObject(transducer);
        const bool added_to_all = added_to_wrapper && digitiser.transducers->setObject(0, transducer);
        transducer->release();
        if (!added_to_all || !digitiser.wrappers->setObject(stylus_wrapper)) {
            stylus_wrapper->release();
            return kIOReturnNoResources;
        }

        stylus_wrapper->release();
    }

    return kIOReturnSuccess;
}

IOReturn VoodooI2CMultitouchHIDEventDriver::publishMultitouchInterface() {
    multitouch_interface = OSTypeAlloc(VoodooI2CMultitouchInterface);

    if (!multitouch_interface ||
        !multitouch_interface->init(NULL) ||
        !multitouch_interface->attach(this))
        goto exit;

    if (!multitouch_interface->start(this)) {
        multitouch_interface->detach(this);
        goto exit;
    }

    multitouch_interface->setProperty(kIOHIDVendorIDKey, getVendorID(), 32);
    multitouch_interface->setProperty(kIOHIDProductIDKey, getProductID(), 32);

    multitouch_interface->setProperty(kIOHIDDisplayIntegratedKey, kOSBooleanFalse);

    multitouch_interface->registerService();

    return kIOReturnSuccess;

exit:
    OSSafeReleaseNULL(multitouch_interface);
    return kIOReturnError;
}

inline void VoodooI2CMultitouchHIDEventDriver::setButtonState(DigitiserTransducerButtonState* state, UInt32 bit, UInt32 value, AbsoluteTime timestamp) {
    if (!state || bit >= (sizeof(UInt32) * 8))
        return;

    const UInt32 button_mask = (1U << bit);
    if (value != 0)
        state->update(state->current.value |= button_mask, timestamp);
    else
        state->update(state->current.value &= ~button_mask, timestamp);
}

void VoodooI2CMultitouchHIDEventDriver::setDigitizerProperties() {
    if (!digitiser.transducers)
        return;

    OSDictionary* properties = OSDictionary::withCapacity(5);
    if (!properties)
        return;

    if (digitiser.contact_count)
        properties->setObject("Contact Count Element", digitiser.contact_count);
    if (digitiser.input_mode)
        properties->setObject("Input Mode Element", digitiser.input_mode);
    if (digitiser.contact_count_maximum)
        properties->setObject("Contact Count Maximum Element", digitiser.contact_count_maximum);
    if (digitiser.primaryButton)
        properties->setObject("Primary Button Element", digitiser.primaryButton);
    if (digitiser.secondaryButton)
        properties->setObject("Secondary Button Element", digitiser.secondaryButton);
    setOSDictionaryNumber(properties, "Transducer Count", digitiser.transducers->getCount());

    setProperty("Digitizer", properties);

    OSSafeReleaseNULL(properties);
}

IOReturn VoodooI2CMultitouchHIDEventDriver::setPowerState(unsigned long whichState, IOService* whatDevice) {
    return kIOPMAckImplied;
}

bool VoodooI2CMultitouchHIDEventDriver::start(IOService* provider) {
    if (!super::start(provider))
        return false;

    work_loop = getWorkLoop();

    if (!work_loop)
        return false;

    work_loop->retain();

    command_gate = IOCommandGate::commandGate(this);
    if (!command_gate) {
        OSSafeReleaseNULL(work_loop);
        return false;
    }

    if (work_loop->addEventSource(command_gate) != kIOReturnSuccess) {
        OSSafeReleaseNULL(command_gate);
        OSSafeReleaseNULL(work_loop);
        return false;
    }

    attached_hid_pointer_devices = OSSet::withCapacity(1);
    if (!attached_hid_pointer_devices) {
        work_loop->removeEventSource(command_gate);
        OSSafeReleaseNULL(command_gate);
        OSSafeReleaseNULL(work_loop);
        return false;
    }

    registerHIDPointerNotifications();

    // Read QuietTimeAfterTyping configuration value (if available)
    OSNumber* quietTimeAfterTyping = OSDynamicCast(OSNumber, getProperty("QuietTimeAfterTyping"));

    if (quietTimeAfterTyping != NULL)
        max_after_typing = quietTimeAfterTyping->unsigned64BitValue() * 1000000;

    return true;
}

IOReturn VoodooI2CMultitouchHIDEventDriver::message(UInt32 type, IOService* provider, void* argument) {
    if (!argument && (type == kKeyboardGetTouchStatus ||
                      type == kKeyboardSetTouchStatus ||
                      type == kKeyboardKeyPressTime))
        return kIOReturnBadArgument;

    switch (type) {
        case kKeyboardGetTouchStatus:
        {
#if DEBUG
            IOLog("%s::getEnabledStatus = %s\n", getName(), ignore_all ? "false" : "true");
#endif
            bool* pResult = (bool*)argument;
            *pResult = !ignore_all;
            break;
        }
        case kKeyboardSetTouchStatus:
        {
            bool enable = *((bool*)argument);
#if DEBUG
            IOLog("%s::setEnabledStatus = %s\n", getName(), enable ? "true" : "false");
#endif
            // ignore_all is true when trackpad has been disabled
            if (enable == ignore_all) {
                // save state, and update LED
                ignore_all = !enable;
            }
            break;
        }
        case kKeyboardKeyPressTime:
        {
            //  Remember last time key was pressed
            key_time = *((uint64_t*)argument);
#if DEBUG
            IOLog("%s::keyPressed = %llu\n", getName(), key_time);
#endif
            break;
        }
    }

    return kIOReturnSuccess;
}

IOReturn VoodooI2CMultitouchHIDEventDriver::setProperties(OSObject * properties) {
    // Listen for property changes we are interested in (instead of reading these during frequent IO events)
    OSDictionary* dict = OSDynamicCast(OSDictionary, properties);

    if (dict != NULL) {
        if (OSCollectionIterator* i = OSCollectionIterator::withCollection(dict)) {
            while (OSSymbol* key = OSDynamicCast(OSSymbol, i->getNextObject())) {
                // System -> Preferences -> Accessibility -> Mouse & Trackpad -> Ignore built-in trackpad when mouse or wireless trackpad is present
                // USBMouseStopsTrackpad
                if (key->isEqualTo("USBMouseStopsTrackpad")) {
                    OSNumber* value = OSDynamicCast(OSNumber, dict->getObject(key));

                    if (value != NULL) {
                        IOLog("%s::setProperties %s = %d\n", getName(), key->getCStringNoCopy(), value->unsigned32BitValue());

                        ignore_mouse = (value->unsigned32BitValue() > 0);

                        // If there are devices connected, update the current ignore status.
                        if (attached_hid_pointer_devices && attached_hid_pointer_devices->getCount() > 0)
                            ignore_all = ignore_mouse;
                    }
                }
            }

            i->release();
        }
    }

    return super::setProperties(properties);
}

void VoodooI2CMultitouchHIDEventDriver::registerHIDPointerNotifications() {
    IOServiceMatchingNotificationHandler notificationHandler = OSMemberFunctionCast(IOServiceMatchingNotificationHandler, this, &VoodooI2CMultitouchHIDEventDriver::notificationHIDAttachedHandler);

    // Determine if we should listen for USB mouse attach events as per configuration
    OSBoolean* isEnabled = OSDynamicCast(OSBoolean, getProperty("ProcessUSBMouseStopsTrackpad"));

    if (isEnabled && isEnabled->isTrue()) {
        // USB mouse HID description as per USB spec: http://www.usb.org/developers/hidpage/HID1_11.pdf
        OSDictionary* matchingDictionary = serviceMatching("IOUSBInterface");
        if (matchingDictionary) {
            propertyMatching(OSSymbol::withCString(kUSBInterfaceClass), OSNumber::withNumber(kUSBHIDInterfaceClass, 8), matchingDictionary);
            propertyMatching(OSSymbol::withCString(kUSBInterfaceSubClass), OSNumber::withNumber(kUSBHIDBootInterfaceSubClass, 8), matchingDictionary);
            propertyMatching(OSSymbol::withCString(kUSBInterfaceProtocol), OSNumber::withNumber(kHIDMouseInterfaceProtocol, 8), matchingDictionary);

            // Register for future services.
            usb_hid_publish_notify = addMatchingNotification(gIOFirstPublishNotification, matchingDictionary, notificationHandler, this, NULL, 10000);
            usb_hid_terminate_notify = addMatchingNotification(gIOTerminatedNotification, matchingDictionary, notificationHandler, this, NULL, 10000);
            OSSafeReleaseNULL(matchingDictionary);
        }
    }

    // Determine if we should listen for bluetooth mouse attach events as per configuration
    isEnabled = OSDynamicCast(OSBoolean, getProperty("ProcessBluetoothMouseStopsTrackpad"));

    if (isEnabled && isEnabled->isTrue()) {
        // Bluetooth HID devices
        OSDictionary* matchingDictionary = serviceMatching("IOBluetoothHIDDriver");
        if (matchingDictionary) {
            propertyMatching(OSSymbol::withCString(kIOHIDVirtualHIDevice), OSBoolean::withBoolean(false), matchingDictionary);

            // Register for future services.
            bluetooth_hid_publish_notify = addMatchingNotification(gIOFirstPublishNotification, matchingDictionary, notificationHandler, this, NULL, 10000);
            bluetooth_hid_terminate_notify = addMatchingNotification(gIOTerminatedNotification, matchingDictionary, notificationHandler, this, NULL, 10000);
            OSSafeReleaseNULL(matchingDictionary);
        }
    }
}

void VoodooI2CMultitouchHIDEventDriver::unregisterHIDPointerNotifications() {
    // Free device matching notifiers
    if (usb_hid_publish_notify) {
        usb_hid_publish_notify->remove();
        OSSafeReleaseNULL(usb_hid_publish_notify);
    }

    if (usb_hid_terminate_notify) {
        usb_hid_terminate_notify->remove();
        OSSafeReleaseNULL(usb_hid_terminate_notify);
    }

    if (bluetooth_hid_publish_notify) {
        bluetooth_hid_publish_notify->remove();
        OSSafeReleaseNULL(bluetooth_hid_publish_notify);
    }

    if (bluetooth_hid_terminate_notify) {
        bluetooth_hid_terminate_notify->remove();
        OSSafeReleaseNULL(bluetooth_hid_terminate_notify);
    }

    OSSafeReleaseNULL(attached_hid_pointer_devices);
}

void VoodooI2CMultitouchHIDEventDriver::notificationHIDAttachedHandlerGated(IOService * newService, IONotifier * notifier) {
    if (!newService || !notifier || !attached_hid_pointer_devices)
        return;

    char path[256] = {0};
    int len = sizeof(path) - 1;
    newService->getPath(path, &len, gIOServicePlane);

    if (notifier == usb_hid_publish_notify) {
        IORegistryEntry* hid_child = OSDynamicCast(IORegistryEntry, newService->getChildEntry(gIOServicePlane));

        if (!hid_child)
            return;

        OSNumber* primary_usage_page = OSDynamicCast(OSNumber, hid_child->getProperty(kIOHIDPrimaryUsagePageKey));
        OSNumber* primary_usage= OSDynamicCast(OSNumber, hid_child->getProperty(kIOHIDPrimaryUsageKey));

        if (!primary_usage_page || !primary_usage)
            return;

        // Ignore a touchscreen only when both its usage page and usage match.
        // The previous AND condition accidentally filtered unrelated devices.
        const bool is_touchscreen =
            primary_usage_page->unsigned32BitValue() == kHIDPage_Digitizer &&
            primary_usage->unsigned32BitValue() == kHIDUsage_Dig_TouchScreen;
        if (!is_touchscreen) {
            attached_hid_pointer_devices->setObject(newService);
            IOLog("%s: USB pointer HID device published: %s, # devices: %d\n", getName(), path, attached_hid_pointer_devices->getCount());
        }
    }

    if (notifier == usb_hid_terminate_notify) {
        attached_hid_pointer_devices->removeObject(newService);
        IOLog("%s: USB pointer HID device terminated: %s, # devices: %d\n", getName(), path, attached_hid_pointer_devices->getCount());
    }

    if (notifier == bluetooth_hid_publish_notify) {
        // Filter on specific CoD (Class of Device) bluetooth devices only
        OSNumber* propDeviceClass = OSDynamicCast(OSNumber, newService->getProperty("ClassOfDevice"));

        if (propDeviceClass != NULL) {
            long classOfDevice = propDeviceClass->unsigned32BitValue();

            long deviceClassMajor = (classOfDevice & 0x1F00) >> 8;
            long deviceClassMinor = (classOfDevice & 0xFF) >> 2;

            if (deviceClassMajor == kBluetoothDeviceClassMajorPeripheral) { // Bluetooth peripheral devices
                long deviceClassMinor1 = (deviceClassMinor) & 0x30;
                long deviceClassMinor2 = (deviceClassMinor) & 0x0F;

                if (deviceClassMinor1 == kBluetoothDeviceClassMinorPeripheral1Pointing || // Seperate pointing device
                    deviceClassMinor1 == kBluetoothDeviceClassMinorPeripheral1Combo) // Combo bluetooth keyboard/touchpad
                {
                    if (deviceClassMinor2 == kBluetoothDeviceClassMinorPeripheral2Unclassified || // Mouse
                        deviceClassMinor2 == kBluetoothDeviceClassMinorPeripheral2DigitizerTablet || // Magic Touchpad
                        deviceClassMinor2 == kBluetoothDeviceClassMinorPeripheral2DigitalPen) // Wacom Tablet
                    {
                        attached_hid_pointer_devices->setObject(newService);
                        IOLog("%s: Bluetooth pointer HID device published: %s, # devices: %d\n", getName(), path, attached_hid_pointer_devices->getCount());
                    }
                }
            }
        }
    }

    if (notifier == bluetooth_hid_terminate_notify) {
        attached_hid_pointer_devices->removeObject(newService);
        IOLog("%s: Bluetooth pointer HID device terminated: %s, # devices: %d\n", getName(), path, attached_hid_pointer_devices->getCount());
    }

    if (notifier == usb_hid_publish_notify || notifier == bluetooth_hid_publish_notify) {
        if (ignore_mouse && attached_hid_pointer_devices->getCount() > 0) {
            // One or more USB or Bluetooth pointer devices attached, disable trackpad
            ignore_all = true;
        }
    }

    if (notifier == usb_hid_terminate_notify || notifier == bluetooth_hid_terminate_notify) {
        if (ignore_mouse && attached_hid_pointer_devices->getCount() == 0) {
            // No USB or bluetooth pointer devices attached, re-enable trackpad
            ignore_all = false;
        }
    }
}

bool VoodooI2CMultitouchHIDEventDriver::notificationHIDAttachedHandler(void * refCon, IOService * newService, IONotifier * notifier) {
    if (!command_gate || !newService || !notifier)
        return false;

    const IOReturn result = command_gate->runAction((IOCommandGate::Action)OSMemberFunctionCast(
        IOCommandGate::Action, this,
        &VoodooI2CMultitouchHIDEventDriver::notificationHIDAttachedHandlerGated),
        newService, notifier);

    return result == kIOReturnSuccess;
}
