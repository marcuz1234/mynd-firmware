// Due to the flash size limit, only the WARNING level is available
// for use in the complete firmware (including the bootloader).
#if defined(BOOTLOADER)
#define LOG_LEVEL LOG_LEVEL_WARNING
#else
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#include <algorithm>
#include <utility>
#include <optional>
#include <functional>

#include "config.h"
#include "board.h"
#include "board_link.h"
#include "bsp_bluetooth_uart.h"
#include "actionslink.h"
#include "task_priorities.h"
#include "logger.h"

#include "ux/audio/audio.h"

#include "external/teufel/libs/GenericThread/GenericThread++.h"
#include "task_audio.h"
#include "task_bluetooth.h"
#include "task_system.h"

#include "external/teufel/libs/property/property.h"
#include "external/teufel/libs/core_utils/mapper.h"
#include "external/teufel/libs/core_utils/overload.h"
#include "external/teufel/libs/core_utils/sync.h"
#include "external/teufel/libs/app_assert/app_assert.h"
#include "gitversion//version.h"
#include "persistent_storage/kvstorage.h"

#ifdef INCLUDE_PRODUCTION_TESTS
#include "external/teufel/libs/tshell/tshell.h"
#endif // INCLUDE_PRODUCTION_TESTS

#define TASK_BLUETOOTH_STACK_SIZE 448
#define QUEUE_SIZE                8

namespace Teufel::Task::Bluetooth
{

namespace Tua = Teufel::Ux::Audio;
namesp...