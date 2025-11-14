#pragma once

// Pull types & probe/init from the existing universal module
#include "jedec_universal_backup.h"

// Only declare the restore API here (no type re-defs)
bool universal_restore_from_sd(const char *path,
                               const jedec_bus_t *bus,
                               bool verify_after_write);
    