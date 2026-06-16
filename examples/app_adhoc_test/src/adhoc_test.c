#include <stdint.h>
#include <string.h>

#include "app.h"
#include "debug.h"
#include "deck.h"
#include "system.h"

// Forward declaration of SPI test function from adhocdeck
extern void uwbTestSpiCommunication(void);

void appMain() {
  DEBUG_PRINT("Adhoc SPI Test App Started\n");

  // Initialize system
  systemWaitStart();

  // Wait for decks to be initialized
  vTaskDelay(M2T(1000));

  DEBUG_PRINT("Testing SPI communication with DW3000...\n");

  // Test SPI communication
  uwbTestSpiCommunication();

  DEBUG_PRINT("SPI test completed. Check debug output above.\n");

  // Keep the app running
  while(1) {
    vTaskDelay(M2T(1000));
    DEBUG_PRINT("App still running...\n");
  }
}