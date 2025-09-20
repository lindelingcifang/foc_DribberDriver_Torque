
#include "communication.h"

#include "zfoc_main.h"
#include "utils.hpp"

#include <cmsis_os.h>
#include <memory>

#include <gpio.h>

#include <type_traits>

uint64_t serial_number;
char serial_number_str[13]; // 12 digits + null termination

void init_communication(void) {
    if (zfoc.config_.enable_can_a) {
        zfoc.can_a.start_server(&hfdcan2);
    }

    if (zfoc.config_.enable_can_b) {
        zfoc.can_b.start_server(&hfdcan3);
    }
}

