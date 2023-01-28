# SPDX-License-Identifier: Apache-2.0

if(CONFIG_INA260)  
#include_directories(.)
target_include_directories(app PRIVATE .)
target_sources(app PRIVATE ina26x_common.c)
target_sources(app PRIVATE ina26x_trigger.c)
target_sources(app PRIVATE ina260.c)
endif()