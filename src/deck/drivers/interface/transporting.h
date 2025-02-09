#ifndef TRANSPORTING_H_
#define TRANSPORTING_H_

#include <stdint.h>
#include "adhocdeck.h"
#include "quic.h"

#define UWB_CHUNK_SLICE_THRESHOLD 1024

/* Socket */
typedef struct {
    TaskHandle_t currTaskHandle;
    uint16_t connectionID;
    uint16_t streamCount;
    uint16_t streamIDs[QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT];
} UWB_Socket_t;

typedef struct {
    uint16_t peer;
    uint16_t connectionId;
    uint16_t streamId;
} UWB_Transport_Info_t;

/* Transport Operations */
/* Client Operations */
UWB_Socket_t *uwbClientSocketCreate(void);
int uwbClientConnect(UWB_Socket_t *socket, UWB_Address_t peer);
int uwbClientSend(UWB_Socket_t *socket, uint8_t *data, uint32_t len);
int uwbClientClose(UWB_Socket_t *socket); // TODO: coding
/* Server Operations */
int uwbServerListen(void);
int uwbServerRead(uint8_t *cache, uint32_t len);
int uwbServerClose(void); // TODO: coding

#endif
// TODO: 切片阈值怎么设置？
