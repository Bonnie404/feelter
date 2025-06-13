/*
 *   Copyright (c) 2023 ACFR-RPG, University of Sydney, Jesse Morris (jesse.morris@sydney.edu.au)
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include "dynosam/dataprovider/DataProvider.hpp"
#include "dynosam/dataprovider/DataInterfacePipeline.hpp"

#include <functional>

#include <glog/logging.h>

namespace dyno {

DataProvider::DataProvider(DataInterfacePipeline* module) {
    CHECK_NOTNULL(module);
    registerImageContainerCallback(std::bind(&DataInterfacePipeline::fillImageContainerQueue, module, std::placeholders::_1));
    CHECK(image_container_callback_);

}

DataProvider::~DataProvider() {
    shutdown();
}

void DataProvider::shutdown() {
    LOG(INFO) << "Shutting down data provider and associated module";
    shutdown_ = true;
}


DataProvider::DataProvider(DataInterfacePipelineImu* module) : DataProvider(dynamic_cast<DataInterfacePipeline*>(module)) {
    CHECK_NOTNULL(module);
    registerImuMultiCallback(std::bind(
        static_cast<void(DataInterfacePipelineImu::*)(const ImuMeasurements&)>(&DataInterfacePipelineImu::fillImuQueue),
        module,
        std::placeholders::_1));
    registerImuSingleCallback(std::bind(
        static_cast<void(DataInterfacePipelineImu::*)(const ImuMeasurement&)>(&DataInterfacePipelineImu::fillImuQueue),
        module,
        std::placeholders::_1));

    CHECK(imu_multi_input_callback_);
    CHECK(imu_single_input_callback_);
}

} //dyno
