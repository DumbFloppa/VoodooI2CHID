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

#define GetReportType(type) \
    ((type <= kIOHIDElementTypeInput_ScanCodes) ? kIOHIDReportTypeInput : \
    (type <= kIOHIDElementTypeOutput) ? kIOHIDReportTypeOutput : \
    (type <= kIOHIDElementTypeFeature) ? kIOHIDReportTypeFeature : -1)

#define super IOHIDEventService
OSDefineMetaClassAndStructors(VoodooI2CMultitouchHIDEventDriver, IOHIDEventService);

AbsoluteTime last_multi_touch_event = 0;

// The result is stored in a UInt32 by every caller, so don't squeeze it through an (signed) int
static UInt32 scientific_pow(UInt32 significand, UInt32 base, SInt32 exponent) {
    UInt32 ret = significand;

    while (exponent > 0) {
        ret *= base;
        exponent--;
    }

    while (exponent < 0) {
        ret /= base;
        exponent++;
    }

    return ret;
}

// Adds "key == value" to an IOService matching dictionary.
// IOService::propertyMatching() does not take over the caller's reference to the value, so the
// temporary symbol is released here and the caller stays responsible for the value it created.
static void addMatchingProperty(OSDictionary* table, const char* key, OSObject* value) {
    const OSSymbol* symbol = OSSymbol::withCString(key);

    if (symbol && value)
        IOService::propertyMatching(symbol, value, table);

    OSSafeReleaseNULL(symbol);
}

void VoodooI2CMultitouchHIDEventDriver::calibrateJustifiedPreferredStateElement(IOHIDElement* element, SInt32 removal_percentage) {
    // The logical range is signed and (range * percentage) can exceed 32 bits, so don't do this in UInt32
    SInt32 sat_min = element->getLogicalMin();
    SInt32 sat_max = element->getLogicalMax();

    SInt32 diff = static_cast<SInt32>(((static_cast<SInt64>(sat_max) - sat_min) * removal_percentage) / 200);

    sat_min += diff;
    sat_max -= diff;

    element->setCalibration(0, 1, sat_min, sat_max);
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

    // Ignore touchpad interaction(s) shortly after typing
    if (now_ns - key_time < max_after_typing)
        return;

    if (!readyForReports() || report_type != kIOHIDReportTypeInput)
        return;

    if (digitiser.contact_count && digitiser.contact_count->getValue()) {
        digitiser.current_contact_count = digitiser.contact_count->getValue();

        UInt8 finger_count = digitiser.fingers ? digitiser.fingers->getCount() : 0;

        if (finger_count) {
            // Round up the result of division by finger_count
            // This is equivalent to ceil(1.0f * digitiser.current_contact_count / finger_count)
            digitiser.report_count = (digitiser.current_contact_count + finger_count - 1) / finger_count;
        } else {
            // Stylus-only digitiser: there are no finger collections to spread the contacts over
            // (and dividing by finger_count would be a division by zero, i.e. a kernel panic)
            digitiser.report_count = 1;
        }

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
    if (!digitiser.transducers || !digitiser.wrappers)
        return;

    VoodooI2CHIDTransducerWrapper* wrapper;
    wrapper = OSDynamicCast(VoodooI2CHIDTransducerWrapper, digitiser.wrappers->getObject(digitiser.current_report - 1));

    if (!wrapper)
        return;

    UInt8 finger_count = digitiser.fingers->getCount();

    if (finger_count) {
        // Check if we are sending the report to the right wrapper, 99% of the time, `digitiser.current_report - 1` will
        // be the correct index but in rare circumstances, it won't be so we should ensure we have the right index
        if (!wrapper->first_identifier)
            return;

        UInt8 first_identifier = wrapper->first_identifier->getValue();

        // Round up the result of division
        // This is equivalent to ceil(1.0f * (first_identifer + 1) / finger_count) - 1
        UInt8 actual_index = (first_identifier + finger_count) / finger_count - 1;

        if (actual_index != digitiser.current_report - 1) {
            wrapper = OSDynamicCast(VoodooI2CHIDTransducerWrapper, digitiser.wrappers->getObject(actual_index));

            if (!wrapper) {
                // Fall back to the original wrapper
                wrapper = OSDynamicCast(VoodooI2CHIDTransducerWrapper, digitiser.wrappers->getObject(digitiser.current_report - 1));

                if (!wrapper)
                    return;
            }
        }
    }

    for (UInt32 i = 0; i < wrapper->transducers->getCount(); i++) {
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
        // The stylus wrapper is the last one
        wrapper = OSDynamicCast(VoodooI2CHIDTransducerWrapper, digitiser.wrappers->getLastObject());

        if (!wrapper) {
            return;
        }

        VoodooI2CDigitiserStylus* stylus = OSDynamicCast(VoodooI2CDigitiserStylus, wrapper->transducers->getObject(0));

        if (!stylus || !stylus->collection) {
            return;
        }

        OSArray* stylus_children = stylus->collection->getChildElements();

        if (!stylus_children) {
            return;
        }

        IOHIDElement* element = OSDynamicCast(IOHIDElement, stylus_children->getObject(0));

        if (element && report_id == element->getReportID()) {
            handleDigitizerTransducerReport(stylus, timestamp, report_id);
        }
    }
}

void VoodooI2CMultitouchHIDEventDriver::handleDigitizerTransducerReport(VoodooI2CDigitiserTransducer* transducer, AbsoluteTime timestamp, UInt32 report_id) {
    bool handled = false;
    bool has_confidence = false;
    bool has_valid = false;
    UInt32 element_index = 0;
    UInt32 element_count = 0;

    if (!transducer->collection)
        return;

    OSArray* child_elements = transducer->collection->getChildElements();

    if (!child_elements)
        return;

    // Only a real stylus transducer has the stylus-specific members (barrel pressure, eraser, ...).
    // The old C-style cast made this pointer non-null for every transducer, so a finger transducer
    // whose collection contained e.g. an Eraser/Invert usage had memory beyond its own object overwritten.
    VoodooI2CDigitiserStylus* stylus = OSDynamicCast(VoodooI2CDigitiserStylus, transducer);

    for (element_index=0, element_count=child_elements->getCount(); element_index < element_count; element_index++) {
        IOHIDElement* element;
        AbsoluteTime element_timestamp;
        bool element_is_current;
        UInt32 usage_page;
        UInt32 usage;
        UInt32 value;

        element = OSDynamicCast(IOHIDElement, child_elements->getObject(element_index));

        if (!element)
            continue;

        element_timestamp = element->getTimeStamp();
        element_is_current = (element->getReportID() == report_id) && (CMP_ABSOLUTETIME(&timestamp, &element_timestamp) == 0);

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
                        handled |= element_is_current;
                        break;
                    }
                    case kHIDUsage_GD_Y:
                    {
                        transducer->coordinates.y.update(element->getValue(), timestamp);
                        transducer->logical_max_y = element->getLogicalMax();
                        handled |= element_is_current;
                        break;
                    }
                    case kHIDUsage_GD_Z:
                    {
                        transducer->coordinates.z.update(element->getValue(), timestamp);
                        transducer->logical_max_z = element->getLogicalMax();
                        handled |= element_is_current;
                        break;
                    }
                }
                break;
            case kHIDPage_Button:
                if (usage == kHIDUsage_Button_1) {
                    // kHIDUsage_Button_1 is for the clickpad surface
                    setButtonState(&transducer->physical_button, 0, value, timestamp);
                } else {
                    // kHIDUsage_Button_2/3 are for external buttons, thus should be reported
                    // through the trackpoint device.
                    transducer->has_secondary_button = true;
                    setButtonState(&transducer->physical_button, (usage - 2), value, timestamp);
                }
                handled |= element_is_current;
                break;
            case kHIDPage_Digitizer:
                switch (usage) {
                    case kHIDUsage_Dig_TransducerIndex:
                    case kHIDUsage_Dig_ContactIdentifier:
                        transducer->secondary_id = value;
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_Touch:
                    case kHIDUsage_Dig_TipSwitch:
                        setButtonState(&transducer->tip_switch, 0, value, timestamp);
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_InRange:
                        transducer->in_range = value != 0;
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_Confidence:
                    case kHIDUsage_Dig_Quality:
                        transducer->confidence.update(element->getValue(), timestamp);
                        handled |= element_is_current;
                        has_confidence = true;
                        break;
                    case kHIDUsage_Dig_TipPressure:
                    case kHIDUsage_Dig_SecondaryTipSwitch:
                    {
                        transducer->tip_pressure.update(element->getValue(), timestamp);
                        transducer->pressure_physical_max = element->getPhysicalMax();
                        handled |= element_is_current;
                        break;
                    }
                    case kHIDUsage_Dig_XTilt:
                        transducer->tilt_orientation.x_tilt.update(element->getScaledFixedValue(kIOHIDValueScaleTypePhysical), timestamp);
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_YTilt:
                        transducer->tilt_orientation.y_tilt.update(element->getScaledFixedValue(kIOHIDValueScaleTypePhysical), timestamp);
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_Azimuth:
                        transducer->azi_alti_orientation.azimuth.update(element->getValue(), timestamp);
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_Altitude:
                        transducer->azi_alti_orientation.altitude.update(element->getValue(), timestamp);
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_Twist:
                        transducer->azi_alti_orientation.twist.update(element->getScaledFixedValue(kIOHIDValueScaleTypePhysical), timestamp);
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_Width:
                        transducer->dimensions.width.update(element->getValue(), timestamp);
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_Height:
                        transducer->dimensions.height.update(element->getValue(), timestamp);
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_DataValid:
                        if (value)
                            transducer->is_valid = true;
                        else
                            transducer->is_valid = false;
                        has_valid = true;
                        handled |= element_is_current;
                        break;
                    case kHIDUsage_Dig_BarrelPressure:
                        if (stylus) {
                            stylus->barrel_pressure.update(element->getScaledFixedValue(kIOHIDValueScaleTypeCalibrated), timestamp);
                            handled |= element_is_current;
                        }
                        break;
                    case kHIDUsage_Dig_BarrelSwitch:
                        if (stylus) {
                            setButtonState(&stylus->barrel_switch, 1, value, timestamp);
                            handled |= element_is_current;
                        }
                        break;
                    case kHIDUsage_Dig_BatteryStrength:
                        if (stylus) {
                            stylus->battery_strength = element->getValue();
                            handled |= element_is_current;
                        }
                        break;
                    case kHIDUsage_Dig_Eraser:
                        if (stylus) {
                            setButtonState(&stylus->eraser, 2, value, timestamp);
                            stylus->invert = value != 0;
                            handled |= element_is_current;
                        }
                        break;
                    case kHIDUsage_Dig_Invert:
                        if (stylus) {
                            stylus->invert = value != 0;
                            handled |= element_is_current;
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

    if (!handled)
        return;
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

    // handleStop() is NOT called when handleStart() fails, so everything created below has to be
    // released here on the failure paths (the multitouch interface used to stay published forever).
    auto cleanup = [this]() {
        if (multitouch_interface) {
            multitouch_interface->stop(this);
            multitouch_interface->detach(this);
            OSSafeReleaseNULL(multitouch_interface);
        }

        OSSafeReleaseNULL(digitiser.transducers);
        OSSafeReleaseNULL(digitiser.wrappers);
        OSSafeReleaseNULL(digitiser.styluses);
        OSSafeReleaseNULL(digitiser.fingers);
    };

    if (should_have_interface && publishMultitouchInterface() != kIOReturnSuccess) {
        // Without the interface nothing could ever be reported, so fail instead of running as a dead driver
        IOLog("%s::%s Could not publish multitouch interface\n", getName(), name);
        return false;
    }

    digitiser.fingers = OSArray::withCapacity(1);
    digitiser.styluses = OSArray::withCapacity(1);
    digitiser.transducers = OSArray::withCapacity(1);

    if (!digitiser.fingers || !digitiser.styluses || !digitiser.transducers) {
        cleanup();
        return false;
    }

    if (parseElements(kHIDUsage_Dig_Any) != kIOReturnSuccess) {
        IOLog("%s::%s Could not parse multitouch elements\n", getName(), name);
        cleanup();
        return false;
    }

    if (!hid_interface->open(this, 0, OSMemberFunctionCast(IOHIDInterface::InterruptReportAction, this, &VoodooI2CMultitouchHIDEventDriver::handleInterruptReport), NULL)) {
        cleanup();
        return false;
    }

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
    UInt32 physical_max = element->getPhysicalMax();
    UInt8 raw_unit_exponent = element->getUnitExponent();

    if (raw_unit_exponent >> 3) {
        raw_unit_exponent = raw_unit_exponent | 0xf0; // Raise the 4-bit int to an 8-bit int
    }

    SInt32 unit_exponent = static_cast<SInt8>(raw_unit_exponent);

    // Scale to 0.01 mm units
    UInt32 unit = element->getUnit();

    if (unit == kHIDUsage_LengthUnitCentimeter) {
        unit_exponent += 3;
    } else if (unit == kHIDUsage_LengthUnitInch) {
        physical_max *= 254;
        unit_exponent += 1;
    }

    physical_max = scientific_pow(physical_max, 10, unit_exponent);

    return physical_max;
}

IOReturn VoodooI2CMultitouchHIDEventDriver::parseDigitizerElement(IOHIDElement* digitiser_element) {
    OSArray* children = digitiser_element->getChildElements();

    if (!children)
        return kIOReturnNotFound;

    // Let's parse the configuration page first
    if (digitiser_element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_DeviceConfiguration)) {
        for (UInt32 i = 0; i < children->getCount(); i++) {
            IOHIDElement* element = OSDynamicCast(IOHIDElement, children->getObject(i));

            if (!element)
                continue;

            if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_Finger)) {
                OSArray* sub_array = element->getChildElements();

                if (!sub_array)
                    continue;

                for (UInt32 j = 0; j < sub_array->getCount(); j++) {
                    IOHIDElement* sub_element = OSDynamicCast(IOHIDElement, sub_array->getObject(j));

                    if (sub_element && sub_element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_DeviceMode))
                        digitiser.input_mode = sub_element;
                }
            }
        }

        return kIOReturnSuccess;
    }

    for (UInt32 i = 0; i < children->getCount(); i++) {
        IOHIDElement* element = OSDynamicCast(IOHIDElement, children->getObject(i));

        if (!element)
            continue;

        if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_Stylus)) {
            digitiser.styluses->setObject(element);
            setProperty("SupportsInk", 1, 32);
            continue;
        }

        if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_Finger)) {
            digitiser.fingers->setObject(element);

            // Let's grab the logical and physical min/max while we are here
            // also the contact identifier
            OSArray* sub_array = element->getChildElements();

            if (!sub_array)
                continue;

            for (UInt32 j = 0; j < sub_array->getCount(); j++) {
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
        if (element->conformsTo(kHIDPage_Button) && element->getUsage() >= kHIDUsage_Button_1 && element->getUsage() <= kHIDUsage_Button_3) {
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
        return kIOReturnNoMemory;

    // A stylus-only digitiser has no finger collections: the finger bookkeeping below would divide by zero
    if (digitiser.contact_count_maximum && digitiser.fingers->getCount()) {
        UInt8 contact_count_maximum = getElementValue(digitiser.contact_count_maximum);

        if (!contact_count_maximum)
            IOLog("%s::%s Device reported a maximum contact count of 0, no finger will be reported\n", getName(), name);

        // Check if maximum contact count divides by digitiser finger count
        if (contact_count_maximum % digitiser.fingers->getCount() != 0) {
            IOLog("%s::%s Unknown digitiser type: got %d finger collections and a %d maximum contact count, ignoring extra fingers\n", getName(), name, digitiser.fingers->getCount(), contact_count_maximum);

            // Remove extra fingers from digitiser's finger array if necessary
            if (contact_count_maximum < digitiser.fingers->getCount()) {
                while (contact_count_maximum % digitiser.fingers->getCount() != 0)
                    digitiser.fingers->removeObject(digitiser.fingers->getCount() - 1);
            }
        }

        int wrapper_count = contact_count_maximum / digitiser.fingers->getCount();

        for (int i = 0; i < wrapper_count; i++) {
            VoodooI2CHIDTransducerWrapper* wrapper = VoodooI2CHIDTransducerWrapper::wrapper();

            if(!digitiser.wrappers->setObject(wrapper)) {
                IOLog("%s::%s Failed to add Transducer Wrapper to transducer array\n", getName(), name);
                OSSafeReleaseNULL(wrapper);
                return kIOReturnNoResources;
            }

            for (UInt32 j = 0; j < digitiser.fingers->getCount(); j++) {
                IOHIDElement* finger = OSDynamicCast(IOHIDElement, digitiser.fingers->getObject(j));

                if (!finger)
                    continue;

                VoodooI2CDigitiserTransducer* transducer = VoodooI2CDigitiserTransducer::transducer(kDigitiserTransducerFinger, finger);

                if (!transducer) {
                    wrapper->release();
                    return kIOReturnNoMemory;
                }

                wrapper->transducers->setObject(transducer);
                digitiser.transducers->setObject(transducer);
                transducer->release();
            }

            VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer, wrapper->transducers->getObject(0));
            OSArray* transducer_children = (transducer && transducer->collection) ? transducer->collection->getChildElements() : nullptr;

            if (transducer_children) {
                for (UInt32 j = 0; j < transducer_children->getCount(); j++) {
                    IOHIDElement* element = OSDynamicCast(IOHIDElement, transducer_children->getObject(j));

                    if (!element)
                        continue;

                    if (element->conformsTo(kHIDPage_Digitizer, kHIDUsage_Dig_ContactIdentifier))
                        wrapper->first_identifier = element;
                }
            }

            wrapper->release();
        }
    }

    // Add stylus as the final wrapper
    if (digitiser.styluses->getCount()) {
        VoodooI2CHIDTransducerWrapper* stylus_wrapper = VoodooI2CHIDTransducerWrapper::wrapper();

        if (!stylus_wrapper)
            return kIOReturnNoMemory;

        digitiser.wrappers->setObject(stylus_wrapper);

        IOHIDElement* stylus = OSDynamicCast(IOHIDElement, digitiser.styluses->getObject(0));
        VoodooI2CDigitiserStylus* transducer = stylus ? VoodooI2CDigitiserStylus::stylus(kDigitiserTransducerStylus, stylus) : nullptr;

        if (!transducer) {
            stylus_wrapper->release();
            return kIOReturnNoMemory;
        }

        stylus_wrapper->transducers->setObject(transducer);
        digitiser.transducers->setObject(0, transducer);
        transducer->release();
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
    UInt32 buttonMask = (1U << bit); // 1U: shifting a signed 1 into bit 31 is undefined behaviour

    if (value != 0)
        state->update(state->current.value |= buttonMask, timestamp);
    else
        state->update(state->current.value &= ~buttonMask, timestamp);
}

void VoodooI2CMultitouchHIDEventDriver::setDigitizerProperties() {
    if (!digitiser.transducers)
        return;

    OSDictionary* properties = OSDictionary::withCapacity(5);

    if (!properties)
        return;

    properties->setObject("Contact Count Element", digitiser.contact_count);
    properties->setObject("Input Mode Element", digitiser.input_mode);
    properties->setObject("Contact Count Maximum Element", digitiser.contact_count_maximum);
    properties->setObject("Primary Button Element", digitiser.primaryButton);
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

    // super::start() has already run handleStart(). The framework does not call stop() for a start()
    // that returns false, so undo it ourselves (stop -> handleStop) if anything below fails.
    auto fail = [&]() -> bool {
        super::stop(provider);
        return false;
    };

    work_loop = getWorkLoop();

    if (!work_loop)
        return fail();

    work_loop->retain();

    command_gate = IOCommandGate::commandGate(this);

    if (!command_gate) {
        return fail();
    }

    if (work_loop->addEventSource(command_gate) != kIOReturnSuccess) {
        IOLog("%s::%s Could not add command gate to the work loop\n", getName(), name);
        return fail();
    }

    attached_hid_pointer_devices = OSSet::withCapacity(1);

    if (!attached_hid_pointer_devices)
        return fail();

    registerHIDPointerNotifications();

    // Read QuietTimeAfterTyping configuration value (if available)
    OSNumber* quietTimeAfterTyping = OSDynamicCast(OSNumber, getProperty("QuietTimeAfterTyping"));

    if (quietTimeAfterTyping != NULL)
        max_after_typing = quietTimeAfterTyping->unsigned64BitValue() * 1000000;

    return true;
}

IOReturn VoodooI2CMultitouchHIDEventDriver::message(UInt32 type, IOService* provider, void* argument) {
    switch (type) {
        case kKeyboardGetTouchStatus:
        {
            if (!argument)
                return kIOReturnBadArgument;

#if DEBUG
            IOLog("%s::getEnabledStatus = %s\n", getName(), ignore_all ? "false" : "true");
#endif
            bool* pResult = (bool*)argument;
            *pResult = !ignore_all;
            break;
        }
        case kKeyboardSetTouchStatus:
        {
            if (!argument)
                return kIOReturnBadArgument;

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
            if (!argument)
                return kIOReturnBadArgument;

            // Remember last time key was pressed
            key_time = *((uint64_t*)argument);
#if DEBUG
            IOLog("%s::keyPressed = %llu\n", getName(), key_time);
#endif
            break;
        }
        default:
            // Don't swallow messages we don't handle ourselves
            return super::message(type, provider, argument);
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

                        // If there are devices connected and automatically switch the current ignore status on/off
                        // (the set is gone once the driver has been stopped, and this can be called at any time)
                        if (attached_hid_pointer_devices && attached_hid_pointer_devices->getCount() > 0) {
                            ignore_all = ignore_mouse;
                        }
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
            // The OSNumbers (and the key symbols, see addMatchingProperty) used to be leaked on every start
            OSNumber* interface_class = OSNumber::withNumber(kUSBHIDInterfaceClass, 8);
            OSNumber* interface_subclass = OSNumber::withNumber(kUSBHIDBootInterfaceSubClass, 8);
            OSNumber* interface_protocol = OSNumber::withNumber(kHIDMouseInterfaceProtocol, 8);

            addMatchingProperty(matchingDictionary, kUSBInterfaceClass, interface_class);
            addMatchingProperty(matchingDictionary, kUSBInterfaceSubClass, interface_subclass);
            addMatchingProperty(matchingDictionary, kUSBInterfaceProtocol, interface_protocol);

            OSSafeReleaseNULL(interface_class);
            OSSafeReleaseNULL(interface_subclass);
            OSSafeReleaseNULL(interface_protocol);

            // Register for future services
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
            // kOSBooleanFalse is a shared constant: it must not be released
            addMatchingProperty(matchingDictionary, kIOHIDVirtualHIDevice, kOSBooleanFalse);

            // Register for future services
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
    // The set is released in handleStop(); a late notification must not touch it
    if (!attached_hid_pointer_devices)
        return;

    char path[256];
    int len = sizeof(path) - 1;
    memset(path, 0, sizeof(path));
    newService->getPath(path, &len, gIOServicePlane);

    if (notifier == usb_hid_publish_notify) {
        IORegistryEntry* hid_child = OSDynamicCast(IORegistryEntry, newService->getChildEntry(gIOServicePlane));

        if (!hid_child)
            return;

        OSNumber* primary_usage_page = OSDynamicCast(OSNumber, hid_child->getProperty(kIOHIDPrimaryUsagePageKey));
        OSNumber* primary_usage= OSDynamicCast(OSNumber, hid_child->getProperty(kIOHIDPrimaryUsageKey));

        if (!primary_usage_page || !primary_usage)
            return;

        // ignore touchscreens
        // (compare the full values: unsigned8BitValue() truncated vendor pages such as 0xFF01 down to 0x01)
        if (primary_usage_page->unsigned32BitValue() != kHIDPage_Digitizer && primary_usage->unsigned32BitValue() != kHIDUsage_Dig_TouchScreen) {
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
    // The gate is gone once the driver has been stopped
    if (!command_gate)
        return true;

    command_gate->runAction(OSMemberFunctionCast(
                            IOCommandGate::Action, this,
                            &VoodooI2CMultitouchHIDEventDriver::notificationHIDAttachedHandlerGated),
                            newService, notifier);

    return true;
}
