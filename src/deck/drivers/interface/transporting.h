#ifndef __TRANSPORTING_H__
#define __TRANSPORTING_H__

#include <stdint.h>
#include "adhocdeck.h"
#include "quic.h"

void transportInit(void);

/* Transport Operations */
/* Client Operations */
int uwbConnectSend(UWB_Address_t peer); /* return connection id */ // TODO: coding
int uwbCreateStreamSend(UWB_Address_t peer, UWB_Connection_ID id); // TODO: coding
int uwbCloseStream(); // TODO: coding
int uwbCloseConnection(); // TODO: coding
/* Server Operations */
int uwbReadStream(); // TODO: coding

#endif
