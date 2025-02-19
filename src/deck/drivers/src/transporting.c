#include <malloc.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "transporting.h"
#include "system.h"
#include "debug.h"

extern QueueHandle_t streamNotifyQueue; // Queue for receiving user's transport information
extern QUIC_Node_t quicClientNode;
extern QUIC_Node_t quicServerNode;
static const int connTimeToWait = 3000; // If the connection is not established within 3 seconds, the connection is considered failed

/* Create a socket like struct, it works like a socket */
UWB_Socket_t *uwbClientSocketCreate(void) {
    UWB_Socket_t *socket = malloc(sizeof(UWB_Socket_t));
    memset(socket, 0, sizeof(UWB_Socket_t));
    if(socket == NULL) {
        return NULL;
    }
    /* socket segments init */
    socket->connectionID = 0;
    socket->streamCount = 0;
    socket->currTaskHandle = xTaskGetCurrentTaskHandle();
    /* open quic client serve */
    quicClientNode.isOpen = true;

    return socket;
}

/*
 * Connect to a peet(server), use blocking mode, if the connection is not established within 3 seconds, the connection is considered failed
 * @param peer: the address of the server
 * @param socket: the socket to connect
 * @return: DWT_SUCCESS if the connection is established, DWT_ERROR if the connection is not established
 */
int uwbClientConnect(UWB_Socket_t *socket, UWB_Address_t peer) {
    quicClientSendConnRequest(peer, 0, socket->currTaskHandle, false);
    uint32_t connectionID = 0;
    if(xTaskGenericNotifyWait(0, 0, 0, &connectionID, connTimeToWait) == pdTRUE) { // Wait for the connection to be established, notified in quic.c quicStateTransport.
        socket->connectionID = connectionID;
        return DWT_SUCCESS;
    } else {
        return DWT_ERROR;
    }
}

/*
 * write data to the socket send buffer, the data will be sliced into several parts according to UWB_CHUNK_SLICE_THRESHOLD, each part will create a stream
 * @param socket: the socket to send data
 * @param data: the data to send
 * @param len: the length of the data
 * @return: DWT_SUCCESS if the data is sent successfully, DWT_ERROR if the data is not sent successfully
 */
int uwbClientSend(UWB_Socket_t *socket, const uint8_t *data, uint32_t len) {
    if(socket == NULL) {
        DEBUG_PRINT("uwbSend: socket is NULL\n");
        return DWT_ERROR;
    }
    if(socket->connectionID == 0) {
        DEBUG_PRINT("uwbSend: socket is not connected\n");
        return DWT_ERROR;
    }
    /* according to len, slice the data into several parts, each part will create a stream */
    const int chunkNum = ((int)len % UWB_CHUNK_SLICE_THRESHOLD) + 1;
    const uint32_t chunkLen = len / chunkNum;
    uint32_t restLen = len;
    uint32_t curPos = 0;
    int prevStreamID = 0;
    int currStreamID = getNextStreamId();
    int nextStreamID = 0;
    for(int i = 0; i < chunkNum; i++) { // create a stream for each chunk
        if (i == chunkNum - 1) {
            nextStreamID = 0;
        } else {
            nextStreamID = getNextStreamId();
        }
        const int res = quicSendStreamCreate(socket->connectionID, currStreamID, prevStreamID, nextStreamID); // will prevent stream number do not overflow
        if(res == DWT_ERROR) {
            DEBUG_PRINT("uwbSend: quicStreamCreate failed\n");
            return DWT_ERROR;
        }
        socket->streamCount++;
        for(int j = 0; j < QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT; j++) { // find an empty slot to store the streamID
            if(socket->streamIDs[j] == 0) {
                socket->streamIDs[j] = currStreamID;
                break;
            }
        }
        /* write data into stream buffer */
        uint32_t writeLen = (restLen > chunkLen) ? chunkLen : restLen;
        if(restLen == writeLen) quicSendStreamWrite(socket->connectionID, currStreamID, data + curPos, writeLen, true);
        else quicSendStreamWrite(socket->connectionID, currStreamID, data + curPos, writeLen, false);

        curPos += writeLen;
        restLen -= writeLen;
        /* let quic node know which stream is open */
        quicTransformInfoAdd(socket->connectionID, currStreamID);
        /* update stream ids */
        prevStreamID = currStreamID;
        currStreamID = nextStreamID;
    }

    return DWT_SUCCESS;
}

/*
 * begin to serve as a server, listen to the connection request
 */
int uwbServerListen() {
    quicServerNode.isOpen = true;
    return DWT_SUCCESS;
}

/*
 * read data from the streams, when the data is ready, the stream will be notified by a queue, once all data is read, the stream will be removed from the queue
 * @param cache: the buffer to store the data
 * @param len: the length of the data to read
 * @return: 'group id' to let the user know which send instruction is read. DWT_SUCCESS if no data to read, DWT_ERROR if the data is not read successfully, maybe the stream is not ready
 */
int uwbServerRead(uint8_t *cache, uint32_t len) {
    if (!quicServerNode.isOpen) {
        DEBUG_PRINT("uwbRead: quic server is not open\n");
        return DWT_ERROR;
    }
    /* use a queue to notify which stream is ready to read */
    UWB_Transport_Info_t socketInfo = {0};
    if (xQueuePeek(streamNotifyQueue, &socketInfo, 0) == pdTRUE) {
        const int res = quicReceiveStreamRead(socketInfo.connectionId, socketInfo.streamId, cache, len);
        if (res == 1) { // all data in a stream group that can be read currently is read, but there may still be some streams behind which the data is currently incomplete and will be read in the future
            xQueueReceive(streamNotifyQueue, &socketInfo, 0);
            quicReceiveStreamClose(socketInfo.connectionId, socketInfo.streamId); // the incoming stream must be the group header and it's data buffer must be integrated
            return socketInfo.streamGroupID; // return the stream group ID to let the user know which send instruction is returned
        } else if (res == 0) { // read all data is not finished
            return socketInfo.streamGroupID; // return the stream group ID to let the user know which send instruction is returned
        } else {
            return DWT_ERROR; // some error occurs
        }
    } else {
        return DWT_SUCCESS; // success but no data to read
    }
}

/*
 * close the socket, release the resources
 * @param socket: the socket to close
 * @return: DWT_SUCCESS if the socket is closed successfully, DWT_ERROR if the socket is not closed successfully
 */
int uwbClientClose(UWB_Socket_t *socket) {
    if(socket == NULL) {
        DEBUG_PRINT("uwbClose: socket is NULL\n");
        return DWT_ERROR;
    }
    if(socket->connectionID == 0) {
        DEBUG_PRINT("uwbClose: socket is not connected\n");
        return DWT_ERROR;
    }
    /* close the connection(mean while close the streams) */
    quicClientConnClose(socket->connectionID);
    /* release the resources */
    free(socket);
    // TODO: send close message to the server

    return DWT_SUCCESS;
}

int uwbServerClose(void) {
    quicServerNode.isOpen = false;

    return DWT_SUCCESS;
}