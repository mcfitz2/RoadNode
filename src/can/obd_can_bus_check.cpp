// Compile check: keeps the ObdManager adapter building in every firmware env.
#include "obd_can_bus.h"

static roadnode::can::ObdCanBus* const obd_can_bus_check = nullptr;
