/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#include "depth_pressure_sensor_driver_test.h"
#include "keller_sensor_driver.h"
#include "keller_serial_interface.h"

#include "gtest/gtest.h"

/**
 * This is the process of test instantiation. First, a new serial interface is
 * created, then it is used to create a kellerSensorDriver object. The latter
 * is passed to the test suite template to instantiate it.
 */
std::shared_ptr<keller::KellerSerialInterface> pSerial =
    std::make_shared<keller::KellerSerialInterface>("/dev/ttyUSB5", 115200);
std::shared_ptr<keller::KellerSensorDriver> pKellerSensorDriver =
    std::make_shared<keller::KellerSensorDriver>(pSerial, 5, 15.0, 0.75);
INSTANTIATE_TEST_SUITE_P(KellerSensorDriverTestGroup, KellerSensorDriverTest,
                         testing::Values(pKellerSensorDriver),
                         testClassification);
