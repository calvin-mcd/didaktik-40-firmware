#pragma once

#include "config_common.h"

#define ENCODERS_PAD_A { B3, B1 }
#define ENCODERS_PAD_B { B2, B0 }

#define AUDIO_PIN B7

#define ACTIVITY_LED_ENABLE D5

#undef LOCKING_SUPPORT_ENABLE
#undef LOCKING_RESYNC_ENABLE

#define NO_ACTION_ONESHOT
#define LAYER_STATE_8BIT
