#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "FreeRTOS.h"
#include "debug.h"
#include "system.h"
#include "quic.h"

#ifndef QUIC_DEBUG_ENABLE
#undef DEBUG_PRINT
#define DEBUG_PRINT
#endif

/* Test */
#define QUIC_TEST_CLIENT
#define QUIC_TEST_SERVER

#define ERROR (-1)
#define SUCCESS 0

/* QUIC Constants */
#define QUIC_DEFAULT_ACK_RANGES_CAPACITY 10

static QueueHandle_t rxPacketQueue;
QueueHandle_t streamNotifyQueue;
static TaskHandle_t quicRxTaskHandle;
static TaskHandle_t quicTxTaskHandle;
QUIC_Node_t quicClientNode;
QUIC_Node_t quicServerNode;
static uint16_t quicSrcConnId = 1;
static uint16_t quicTempSrcConnId = 0; /* There can only be one at the same time */
static SemaphoreHandle_t quicConnIdMutex;
static uint16_t quicStreamId = 1;
static SemaphoreHandle_t quicStreamIdMutex;
static uint16_t quicStreamGroupId = 1;

/* ID get */
static uint16_t getNextSrcConnId() {
    xSemaphoreTake(quicConnIdMutex, M2T(0));
    const uint16_t nextId = quicSrcConnId++;
    xSemaphoreGive(quicConnIdMutex);
    return nextId;
}

uint16_t getNextStreamId() {
    xSemaphoreTake(quicStreamIdMutex, M2T(0));
    const uint16_t nextId = quicStreamId++;
    xSemaphoreGive(quicStreamIdMutex);
    return nextId;
}

static uint16_t getNextStreamGroupId() {
    const uint16_t nextId = quicStreamGroupId++;
    return nextId;
}

/* Global buffer operations */
static DataBlock_t *getStreamBufferDataBlock(void *streamBuffer_, const uint32_t minSize, const QUIC_NODE_STATUS status) {
    if(status == QUIC_SERVER) {
        QUIC_Stream_Receive_Buffer_t *streamBuffer = streamBuffer_;
        if(streamBuffer->freeBlocksHead->next) { // check if there is a free block
            DataBlock_t *prev = streamBuffer->freeBlocksHead;
            DataBlock_t *curr = streamBuffer->freeBlocksHead->next;

            while(curr) {
                if(curr->capacity >= minSize) {
                    // if(prev) prev->next = curr->next;
                    // else streamBuffer->freeBlocks = curr->next;
                    prev->next = curr->next;
                    curr->next = NULL;
                    return curr;
                }
                prev = curr;
                curr = curr->next;
            }
        }
    }
    else if (status == QUIC_CLIENT) {
        QUIC_Stream_Send_Buffer_t *streamBuffer = streamBuffer_;
        if(streamBuffer->freeBlocksHead->next) {
            DataBlock_t *prev = streamBuffer->freeBlocksHead;
            DataBlock_t *curr = streamBuffer->freeBlocksHead->next;

            while(curr) {
                if(curr->capacity >= minSize) {
                    // if(prev) prev->next = curr->next;
                    // else streamBuffer->freeBlocks = curr->next;
                    prev->next = curr->next;
                    curr->next = NULL;
                    return curr;
                }
                prev = curr;
                curr = curr->next;
            }
        }
    }
    return dataBlockInit(minSize);
}

/* Send buffer operations */
/* Find a free block to cache data, and add it to the pending list, used by application */
static int writeStreamSendBuffer(QUIC_Stream_Send_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength, bool isFin) { // TODO: modify, add fin
    if(streamBuffer->sendOffset + dataLength > streamBuffer->maxSendOffset) { // overflow
        DEBUG_PRINT("writeStreamSendBuffer: maxSendOffset exceeded\n");
        return ERROR;
    }
    DataBlock_t *block = getStreamBufferDataBlock(streamBuffer, dataLength, QUIC_CLIENT);
    if(block == NULL) {
        DEBUG_PRINT("writeStreamSendBuffer: no block available\n"); // space is not enough
        return ERROR;
    }
    /* copy the data to the block */
    memcpy(block->data, data, dataLength);
    block->length = dataLength;
//    block->capacity = QUIC_STREAM_DATA_BLOCK_MAX_DATA_SIZE;
    block->offset = streamBuffer->maxSendOffset;
    block->isFin = isFin;
    /* add the block to the send list */
    if(streamBuffer->pendingBlockList.head == NULL) {
        streamBuffer->pendingBlockList.head = block;
    } else {
        streamBuffer->pendingBlockList.tail->next = block;
    }
    streamBuffer->pendingBlockList.tail = block;
    streamBuffer->pendingBlockList.count++;
    streamBuffer->maxSendOffset += dataLength;
    return (int)dataLength;
}
/* when stream data send, we need to get data from send buffer, and load it to packet */
static int readStreamSendBuffer(QUIC_Stream_Send_Buffer_t *streamBuffer, QUIC_Stream_Sending_Data_t *sendingData, const uint32_t dataLength, const uint32_t packetNumber) {
    if(streamBuffer == NULL || sendingData == NULL || dataLength == 0) {
        DEBUG_PRINT("readStreamSendBuffer: data not complement.\n");
        return ERROR;
    }
    if(streamBuffer->pendingBlockList.head == NULL) {
        DEBUG_PRINT("readStreamSendBuffer: no data need to send.\n");
        return ERROR;
    }
    DataBlock_t *block = streamBuffer->pendingBlockList.head;
    uint32_t blockOffset = 0;
    /* if the block is partially sent, the offset needs to be calculated */
    if(block->offset + block->length > streamBuffer->sendOffset) blockOffset = streamBuffer->sendOffset - block->offset;
    /* calculate the length of data that can be sent at this time */
    const uint32_t available = block->length - blockOffset;
    const uint32_t sendLength = available < dataLength ? available : dataLength;
    /* write sending data struct */
    sendingData->data = block->data + blockOffset;
    sendingData->length = sendLength;
    sendingData->offset = streamBuffer->sendOffset;
    sendingData->packetNumber = packetNumber;
    sendingData->isFin = block->isFin;
    /* remove to the unacked block list */
    if(blockOffset + sendLength >= block->length) { // TODO: there block offset may has some bug
        /* entire block send ready */
        streamBuffer->pendingBlockList.head = block->next;
        if(streamBuffer->pendingBlockList.head == NULL) streamBuffer->pendingBlockList.tail = NULL;
        streamBuffer->pendingBlockList.count--;
        /* remove to unacked block list */
        block->next = NULL;
        if(streamBuffer->unackedBlockList.head == NULL) {
            streamBuffer->unackedBlockList.head = block;
            streamBuffer->unackedBlockList.tail = block;
        }
        else {
            streamBuffer->unackedBlockList.tail->next = block;
            streamBuffer->unackedBlockList.tail = block;
        }
        streamBuffer->unackedBlockList.count++;
    }
    else {
        /* only part of the block is sent, and a new block needs to be created */
        DataBlock_t *newBlock = getStreamBufferDataBlock(streamBuffer,sendLength, QUIC_CLIENT);
        if(newBlock == NULL) {
            DEBUG_PRINT("readStreamSendBuffer: can not malloc new block, space is not enough.\n");
            return ERROR;
        }
        memcpy(newBlock->data, block->data + blockOffset, sendLength);
        newBlock->length = sendLength;
        newBlock->offset = streamBuffer->sendOffset;
        /* add to unacked block list */
        newBlock->next = NULL;
        if(streamBuffer->unackedBlockList.head == NULL) {
            streamBuffer->unackedBlockList.head = newBlock;
            streamBuffer->unackedBlockList.tail = newBlock;
        }
        else {
            streamBuffer->unackedBlockList.tail->next = newBlock;
            streamBuffer->unackedBlockList.tail = newBlock;
        }
        streamBuffer->unackedBlockList.count++;
    }
    streamBuffer->sendOffset += sendLength; // TODO: modify, when the block is resending packet, the send offset is not right
    return (int)sendLength;
}

static int ackStreamSendBuffer(QUIC_Send_Stream_Item_t *streamItem, const uint32_t offset, const uint32_t length) {
    if(streamItem == NULL) {
        DEBUG_PRINT("ackStreamSendBuffer: stream item is not exist.\n");
        return ERROR;
    }
    QUIC_Stream_Send_Buffer_t *streamBuffer = &streamItem->dataBuffer;
    DataBlock_t *currBlock = streamBuffer->unackedBlockList.head;
    DataBlock_t *prevBlock = NULL;
    while (currBlock != NULL) {
        /* check if it is acked block */
        if (currBlock->offset == offset && currBlock->length == length) {
            if (prevBlock != NULL) prevBlock->next = currBlock->next;
            else streamBuffer->unackedBlockList.head = currBlock->next;
            if (currBlock == streamBuffer->unackedBlockList.tail) streamBuffer->unackedBlockList.tail = prevBlock;
            streamBuffer->unackedBlockList.count--;
            /* remove the block to free list */
            currBlock->next = streamBuffer->freeBlocksHead->next;
            streamBuffer->freeBlocksHead->next = currBlock;
            /* check if all data in this stream is acked, if so, change state machine and close the stream */
            if (streamBuffer->unackedBlockList.count == 0 && streamBuffer->pendingBlockList.count == 0) { // all data is acked
                streamItem->sendingStatus = QUIC_STREAM_SENDING_DATA_RECEIVED;
            }
            return SUCCESS;
        }
        prevBlock = currBlock;
        currBlock = currBlock->next;
    }
    DEBUG_PRINT("ackStreamSendBuffer: the acked block can not found.\n");
    return ERROR;
}

static int toResendStreamSendBuffer(QUIC_Stream_Send_Buffer_t *streamBuffer, const uint32_t offset, const uint32_t length) { // TODO: modify
    if(streamBuffer == NULL) {
        DEBUG_PRINT("toResendStreamSendBuffer: data not complement.\n");
        return ERROR;
    }
    DataBlock_t *currBlock = streamBuffer->unackedBlockList.head;
    DataBlock_t *prevBlock = NULL;
    while (currBlock != NULL) {
        DataBlock_t *toResendBlock = NULL;
        /* find block */
        if (currBlock->offset == offset && currBlock->length == length) {
            if (prevBlock != NULL) prevBlock->next = currBlock->next;
            else streamBuffer->unackedBlockList.head = currBlock->next;
            if (currBlock == streamBuffer->unackedBlockList.tail) streamBuffer->unackedBlockList.tail = prevBlock;
            streamBuffer->unackedBlockList.count--;
            /* remove the block to pending list */
            currBlock->next = streamBuffer->pendingBlockList.head;
            streamBuffer->pendingBlockList.head = currBlock;
            if (streamBuffer->pendingBlockList.tail == NULL) streamBuffer->pendingBlockList.tail = currBlock;
            streamBuffer->pendingBlockList.count++;
            /* update send offset */
            if (currBlock->offset + currBlock->length < streamBuffer->sendOffset) streamBuffer->sendOffset = currBlock->offset;

            return SUCCESS;
        }
        prevBlock = currBlock;
        currBlock = currBlock->next;
    }
    DEBUG_PRINT("toResendStreamSendBuffer: the acked block can not found.\n");
    return ERROR;
}

/* Receive buffer operations */
/* Read data from the reception buffer, and put the data to the application cache */
static int readStreamReceiveBuffer(QUIC_Stream_Receive_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength) {
//    if(streamBuffer->readOffset >= streamBuffer->consumedOffset) {
//        DEBUG_PRINT("writeStreamReceiveBuffer: no data can read\n");
//        return ERROR;
//    }
    uint32_t totalReadLen = 0;
    const DataBlock_t *curr = streamBuffer->receiveBlockList.head;
    /* Find current offset block */
    while(curr && curr->offset + curr->length <= streamBuffer->readOffset) curr = curr->next;
    /* Read data from receive buffer */
    while(curr && totalReadLen < dataLength) {
        const uint32_t blockOffset = streamBuffer->readOffset - curr->offset;
        const uint32_t availableLen = curr->length - blockOffset;
        const uint32_t readLen = MIN(availableLen, dataLength - totalReadLen);
        memcpy((void *)data + totalReadLen, curr->data + blockOffset, readLen);
        totalReadLen += readLen;
        streamBuffer->readOffset += readLen;
        if(blockOffset + readLen >= curr->length) curr = curr->next;
    }
//    if (curr->isFin) return 1; // read buffer is finished
//    else return 0; // read buffer successfully, but is not finished
    return (int)totalReadLen;
}

static int updateStreamReceiveBufferConsumedOffset(QUIC_Stream_Receive_Buffer_t *streamBuffer) {
    if(streamBuffer == NULL || streamBuffer->receiveBlockList.head == NULL) { // buffer or block is null
        DEBUG_PRINT("updateConsumedOffset: stream buffer is null.\n");
        return ERROR;
    }
    uint32_t currOffset = streamBuffer->consumedOffset;
    DataBlock_t *currBlock = streamBuffer->receiveBlockList.head;
    DataBlock_t * prevBlock = NULL;
    while(currBlock != NULL) {
        if (currBlock->offset > currOffset) {
            DEBUG_PRINT("updateConsumedOffset: the current block is not continuous with the consumed data.\n");
            return ERROR;
        }
        /* update consumed offset */
        const uint32_t currBlockOffsetEnd = currBlock->offset + currBlock->length;
        if(currBlockOffsetEnd > currOffset) currOffset = currBlockOffsetEnd;
        /* release the consumed block */
        if(streamBuffer->readOffset >= currBlockOffsetEnd) {
            /* remove from the reception block list */
            if(prevBlock != NULL) prevBlock->next = currBlock->next;
            else streamBuffer->receiveBlockList.head = currBlock->next;
            if(currBlock == streamBuffer->receiveBlockList.tail) streamBuffer->receiveBlockList.tail = prevBlock;
            streamBuffer->receiveBlockList.count--;
            /* move the released block to the free list */
            DataBlock_t *freeBlock = currBlock;
            currBlock = currBlock->next;
            freeBlock->next = streamBuffer->freeBlocksHead->next;
            streamBuffer->freeBlocksHead = freeBlock;
            freeBlock->length = 0;
        }
        else {
            prevBlock = currBlock;
            currBlock = currBlock->next;
        }
    }
    streamBuffer->consumedOffset = currOffset;
    return SUCCESS;
}

static int mergeStreamReceiveBufferAdjacentBlocks(QUIC_Stream_Receive_Buffer_t *streamBuffer) {
    if(streamBuffer == NULL || streamBuffer->receiveBlockList.head == NULL) { // buffer or block is null
        DEBUG_PRINT("mergeReceiveAdjacentBlocks: stream buffer is null.\n");
        return ERROR;
    }
    DataBlock_t *currBlock = streamBuffer->receiveBlockList.head;
    while(currBlock != NULL && currBlock->next != NULL) {
        DataBlock_t *nextBlock = currBlock->next;
        /* check adjacent */
        if(currBlock->offset + currBlock->length == nextBlock->offset) {
            if(currBlock->capacity >= currBlock->length + nextBlock->length) {
                memcpy(currBlock->data + currBlock->length, nextBlock->data, nextBlock->length);
                currBlock->length += nextBlock->length;
                /* remove next block to the free block list */
                currBlock->next = nextBlock->next;
                if(nextBlock == streamBuffer->receiveBlockList.tail) streamBuffer->receiveBlockList.tail = currBlock;
                streamBuffer->receiveBlockList.count--;
                nextBlock->next = streamBuffer->freeBlocksHead->next;
                streamBuffer->freeBlocksHead->next = nextBlock;
                nextBlock->length = 0;
            }
        }
        currBlock = currBlock->next;
    }
    return SUCCESS;
}

static int verifyStreamDataIntegrity(QUIC_Stream_Receive_Buffer_t *streamBuffer) {
    if(streamBuffer == NULL || streamBuffer->receiveBlockList.head == NULL) { // buffer or block is null
        DEBUG_PRINT("verifyStreamDataIntegrity: stream buffer is null.\n");
        return ERROR;
    }
    DataBlock_t *currBlock = streamBuffer->receiveBlockList.head;
    uint32_t expectedOffset = 0;
    bool finFound = false;

    while (currBlock != NULL) {
        if(currBlock->offset != expectedOffset) { // some packet lost, so now, the stream's status should be QUIC_STREAM_RECEIVING_SIZE_KNOWN, waiting for retransmission
            DEBUG_PRINT("verifyStreamDataIntegrity: data integrity error, offset not match.\n");
            return ERROR;
        }
        expectedOffset += currBlock->length;
        /* check fin */
        if(currBlock->isFin) {
            if(finFound) {
                DEBUG_PRINT("verifyStreamDataIntegrity: data integrity error, multiple fin found.\n");
                return ERROR;
            }
            finFound = true;
        }
        currBlock = currBlock->next;
    }

    /* make sure to end with a fin block */
    if(!finFound) {
        DEBUG_PRINT("verifyStreamDataIntegrity: data integrity error, no fin found.\n");
        return ERROR;
    }
    return SUCCESS;
}

static int writeStreamReceiveBuffer(QUIC_Stream_Receive_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength, const uint32_t offset, const bool isFin) {
//    if(offset + dataLength > streamBuffer->maxReceiveOffset) { // exceed max receive window
//        DEBUG_PRINT("writeStreamReceiveBuffer: maxReceiveOffset exceeded\n");
//        return ERROR;
//    }
    DataBlock_t *block = getStreamBufferDataBlock(streamBuffer, dataLength, QUIC_SERVER);
    if(block == NULL) { // no more space
        DEBUG_PRINT("writeStreamReceiveBuffer: no block available\n");
        return ERROR;
    }
    memcpy(block->data, data, dataLength);
    block->length = dataLength;
    block->offset = offset;
    block->isFin = isFin;
    /* insert to the list by order */
    DataBlock_t *prev = NULL;
    DataBlock_t *curr = streamBuffer->receiveBlockList.head;
    while(curr && curr->offset <= offset) {
        prev = curr;
        curr = curr->next;
    }
    if(prev != NULL) {
        block->next = prev->next;
        prev->next = block;
    } else {
        block->next = streamBuffer->receiveBlockList.head;
        streamBuffer->receiveBlockList.head = block;
    }
    if(block->next == NULL) streamBuffer->receiveBlockList.tail = block;
    streamBuffer->receiveBlockList.count++;
    /* merge blocks to optimize memory */
    mergeStreamReceiveBufferAdjacentBlocks(streamBuffer);
    /* update and release stored blocks */
    updateStreamReceiveBufferConsumedOffset(streamBuffer);

    return (int)dataLength;
}

static int removeStreamReceiveBuffer(QUIC_Read_Stream_Item_t *readStream) {
    if (readStream == NULL) {
        DEBUG_PRINT("removeStreamReceiveBuffer: readStream is null.\n");
        return ERROR;
    }
    if (readStream->receivingStatus != QUIC_STREAM_RECEIVING_DATA_READY) {
        DEBUG_PRINT("removeStreamReceiveBuffer: stream is not ready, can not remove.\n");
        return ERROR;
    }
    /* remove buffer from stream item */
    QUIC_Stream_Receive_Buffer_t *streamBuffer = &readStream->dataBuffer;
    /* free receive block list */
    DataBlock_t *currBlock = streamBuffer->receiveBlockList.head;
    while (currBlock != NULL) {
        DataBlock_t *freeBlock = currBlock;
        currBlock = currBlock->next;
        freeBlock->next = streamBuffer->freeBlocksHead->next;
        streamBuffer->freeBlocksHead->next = freeBlock;
        freeBlock->length = 0;
    }
    streamBuffer->receiveBlockList.head = NULL;
    streamBuffer->receiveBlockList.tail = NULL;
    streamBuffer->receiveBlockList.count = 0;

    return SUCCESS;
}

/* Packet info operations */
static int quicPacketInfoNodeInit(QUIC_Packet_Info_Node_t *packetInfoNode, const Quic_One_RTT_Packet_t *oneRTTPacket, const QUIC_Send_Stream_Item_t *sendStreamItem) {
    if(packetInfoNode == NULL || oneRTTPacket == NULL || sendStreamItem == NULL) {
        DEBUG_PRINT("quicPacketInfoNodeInit: data not complement.\n");
        return ERROR;
    }
    packetInfoNode->packetNumber = oneRTTPacket->header.packetNumber;
    packetInfoNode->streamId = sendStreamItem->streamId;
    packetInfoNode->isAcked = false;
    packetInfoNode->isLost = false;
    packetInfoNode->sendTime = xTaskGetTickCount();
    packetInfoNode->retransmitCount = 0;
    const QUIC_Client_Conn_Item_t *connItem = (QUIC_Client_Conn_Item_t*)sendStreamItem->connItemPtr;
    packetInfoNode->connId = connItem->connId;
    packetInfoNode->retransmitPeriod = connItem->retransmitPeriod;
    packetInfoNode->dataBlock = sendStreamItem->dataBuffer.unackedBlockList.tail;
    packetInfoNode->offset = packetInfoNode->dataBlock->offset;
    packetInfoNode->length = packetInfoNode->dataBlock->length;

    return SUCCESS;
}

static int quicPacketInfoNodeInsert(RBRoot_t *packetInfoRBTree, QUIC_Packet_Info_Node_t *packetInfoNode) {
    if(packetInfoRBTree == NULL || packetInfoNode == NULL) {
        DEBUG_PRINT("quicPacketInfoNodeInsert: data not complement.\n");
        return ERROR;
    }
    QUIC_Packet_Info_Manager_t *manager = packetInfoRBTree->externResourcePtr;
    if(packetInfoNode->packetNumber <= manager->minimumUnackedPacketNumber) {
        DEBUG_PRINT("quicPacketInfoNodeInsert: packet number is less than minimum packet acked number.\n");
        return SUCCESS;
    }
    const int res = insertRBTree(manager->packetInfoRBTree, (int)packetInfoNode->packetNumber, packetInfoNode, sizeof(QUIC_Packet_Info_Node_t));
    if(res == ERROR) {
        DEBUG_PRINT("quicPacketInfoNodeInsert: insert failed.\n");
        return ERROR;
    }
    if(packetInfoNode->packetNumber > manager->largestPacketNumber) manager->largestPacketNumber = packetInfoNode->packetNumber;
    manager->packetInfoNodeCount++;

    return SUCCESS;
}

static int quicPacketInfoNodeDelete(RBRoot_t *packetInfoRBTree, uint32_t packetNumber, uint16_t connId) {
    if (packetInfoRBTree == NULL) {
        DEBUG_PRINT("quicPacketInfoNodeDelete: packetInfoRBTree is null.\n");
        return ERROR;
    }
    QUIC_Packet_Info_Manager_t *manager = packetInfoRBTree->externResourcePtr;
    /* get packet info node and delete relative buffer's block */
    const RBNode_t *rbNode = searchRBTree(manager->packetInfoRBTree, (int)packetNumber);
    if(rbNode == NULL) {
        DEBUG_PRINT("quicPacketInfoNodeDelete: packet info node not found.\n");
        return ERROR;
    }
    QUIC_Packet_Info_Node_t *packetInfoNode = rbNode->data;
    /* get connection item */
    QUIC_Client_Conn_Item_t *clientConnItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connId);
    /* get stream item */
    QUIC_Send_Stream_Item_t *sendStreamItem = clientConnItem->sendStreams.streamItemGet(&clientConnItem->sendStreams.streamsMap, packetInfoNode->streamId);
    /* remove the block from the unacked block list */
    ackStreamSendBuffer(sendStreamItem, packetInfoNode->offset, packetInfoNode->length);

    /* delete packet info node */
    int res = deleteRBTree(manager->packetInfoRBTree, (int)packetNumber);
    if(res == -1) {
        DEBUG_PRINT("quicPacketInfoNodeDelete: delete failed.\n");
        return ERROR;
    }
    if(packetNumber == manager->minimumUnackedPacketNumber) {
        packetInfoNode = NULL; // reuse
        res = RBTreeMinimum(manager->packetInfoRBTree, (void **)&packetInfoNode);
        if (res == -1) {
            DEBUG_PRINT("quicPacketInfoNodeDelete: get min packet number failed.\n");
            return ERROR;
        }
        if (res == 1) {
            DEBUG_PRINT("quicPacketInfoNodeDelete: no packet in the tree.\n"); // this means all packet acked
            manager->minimumUnackedPacketNumber = 0; // all packet acked
            return SUCCESS;
        }
        if (packetInfoNode == NULL) {
            DEBUG_PRINT("quicPacketInfoNodeDelete: packetInfoNodePtr is null.\n");
            return ERROR;
        }
        manager->minimumUnackedPacketNumber = packetInfoNode->packetNumber;
    }
    /* check if this stream is finished(receive all data ack in this stream) */
    if (sendStreamItem->sendingStatus == QUIC_STREAM_SENDING_DATA_RECEIVED) {
        quicSendStreamClose(clientConnItem->connId, sendStreamItem->streamId); // close the stream, free it's space
    }

    return SUCCESS;
}

static int quicPacketInfoManagerInit(QUIC_Packet_Info_Manager_t *packetInfoManager) {
    if (packetInfoManager == NULL) {
        DEBUG_PRINT("quicPacketInfoManagerInit: packetInfoManager is null.\n");
        return ERROR;
    }
    packetInfoManager->largestPacketNumber = 0; // no packet
    packetInfoManager->minimumUnackedPacketNumber = 0; // all packet acked
    packetInfoManager->packetInfoNodeCount = 0;
    packetInfoManager->packetInfoRBTree = createRBTree();
    packetInfoManager->packetInfoRBTree->externResourcePtr = packetInfoManager;

    packetInfoManager->packetInfoNodeInsert = quicPacketInfoNodeInsert;
    packetInfoManager->packetInfoNodeDelete = quicPacketInfoNodeDelete;

    return SUCCESS;
}

/* ACK Ranges operations */
static int quicACKRangesMoveToFree(QUIC_ACK_Ranges_t *ackRanges, QUIC_ACK_Ranges_Block_t *block) {
    if(block == NULL) {
        DEBUG_PRINT("quicACKRangesMoveToFree: data not complement.\n");
        return ERROR;
    }
    block->ackRangeLength = 0;
    block->gap = 0;
    if(ackRanges->freeBlocks == NULL) {
        block->next = NULL;
        ackRanges->freeBlocks = block;
    } else {
        block->next = (struct QUIC_ACK_Ranges_Block_t*)ackRanges->freeBlocks;
        ackRanges->freeBlocks = block;
    }
    return SUCCESS;
}

static QUIC_ACK_Ranges_Block_t* quicACKRangesBlockGet(QUIC_ACK_Ranges_t *ackRanges) {
    if (ackRanges->freeBlocks == NULL) {
        QUIC_ACK_Ranges_Block_t *newBlock = malloc(sizeof(QUIC_ACK_Ranges_Block_t));
        ackRanges->ackRangeCapacity++;
        if(newBlock == NULL) {
            DEBUG_PRINT("quicACKRangesAdd: malloc failed.\n");
            return NULL;
        }
        newBlock->ackRangeLength = 0;
        newBlock->gap = 0;
        newBlock->next = (struct QUIC_ACK_Ranges_Block_t*)ackRanges->freeBlocks;
        ackRanges->freeBlocks = newBlock;
    }
    QUIC_ACK_Ranges_Block_t *block = ackRanges->freeBlocks;
    ackRanges->freeBlocks = (QUIC_ACK_Ranges_Block_t *) block->next;
    block->next = NULL;
    return block;
}

static int quicACKRangesInit(QUIC_ACK_Ranges_t **ackRangesPtr, const uint16_t defaultACKRangeCapacity) {
    if(ackRangesPtr == NULL) {
        DEBUG_PRINT("quicACKRangesInit: ackRanges is null.\n");
        return ERROR;
    }
    QUIC_ACK_Ranges_t * ackRanges = malloc(sizeof(QUIC_ACK_Ranges_t)); // TODO: may debug
    ackRanges->freeBlocks = NULL;
    ackRanges->ackRangesList.head = NULL;
    ackRanges->ackRangesList.tail = NULL;
    ackRanges->ackRangeCapacity = defaultACKRangeCapacity;
    ackRanges->freeBlocks = (QUIC_ACK_Ranges_Block_t *)malloc(sizeof(QUIC_ACK_Ranges_Block_t) * defaultACKRangeCapacity);
    if(ackRanges->freeBlocks == NULL) {
        DEBUG_PRINT("quicACKRangesInit: malloc failed.\n");
        return ERROR;
    }
    ackRanges->freeBlocks[0].ackRangeLength = 0;
    ackRanges->freeBlocks[0].gap = 0;
    ackRanges->freeBlocks[0].next = NULL;
    for(int i = 1; i < defaultACKRangeCapacity; i++) {
        ackRanges->freeBlocks[i].ackRangeLength = 0;
        ackRanges->freeBlocks[i].gap = 0;
        ackRanges->freeBlocks[i - 1].next = (struct QUIC_ACK_Ranges_Block_t *) &ackRanges->freeBlocks[i];
    }
    // set ptr value for ackRangesPtr
    *ackRangesPtr = ackRanges;
    // return
    return SUCCESS;
}

static int quicACKRangesAddGap(QUIC_ACK_Ranges_t *ackRanges, const uint16_t gap) {
    if (gap == 0) {
        DEBUG_PRINT("quicACKRangesAddGap: gap is 0.\n");
        return SUCCESS;
    }
    if (gap > 100) { // in most case, it can not be more than 100
        DEBUG_PRINT("quicACKRangesAddGap: gap is too large.\n");
        return ERROR;
    }
    if(ackRanges == NULL) {
        DEBUG_PRINT("quicACKRangesAdd: ackRanges is null.\n");
        return ERROR;
    }
    if(ackRanges->ackRangesList.head == NULL) {
        // get an empty block
        QUIC_ACK_Ranges_Block_t *block = quicACKRangesBlockGet(ackRanges);
        block->gap = gap;
        block->ackRangeLength = 0;
        block->next = NULL;
        // add to the list
        ackRanges->ackRangesList.head = block;
        ackRanges->ackRangesList.tail = block;
        ackRanges->ackRangesList.ackRangeCount++;
        // return
        return SUCCESS;
    }
    /* get the newest block */
    QUIC_ACK_Ranges_Block_t *currBlock = ackRanges->ackRangesList.tail; // in the tail, situation gap = 0, len > 0, will never happen, since quicACKRangesUpdateLen will handle it
    if (currBlock->ackRangeLength == 0) currBlock->gap++;
    else { // since |gap|len|gap|len|, the last block have len, so we need create a new block
        // get an empty block
        QUIC_ACK_Ranges_Block_t *newBlock = quicACKRangesBlockGet(ackRanges);
        newBlock->gap = gap;
        newBlock->ackRangeLength = 0;
        newBlock->next = NULL;
        // add to the list
        ackRanges->ackRangesList.tail->next = (struct QUIC_ACK_Ranges_Block_t*)newBlock;
        ackRanges->ackRangesList.tail = newBlock;
        ackRanges->ackRangesList.ackRangeCount++;
    }
    // return
    return SUCCESS;
}

/* Find relative ack ranges block, and update it, including merge two blocks, split new block, and move empty block to free list.
 * In the end, it will return the number minimum ack need to add.
 * @packetNumberOffset: the offset of the packet number in the ack ranges list, in other words, the packet number - (minimum packet number + 1)
 */
static int quicACKRangesUpdateLen(QUIC_ACK_Ranges_t *ackRanges, const uint16_t packetNumberOffset) {
    if(ackRanges == NULL) {
        DEBUG_PRINT("quicACKRangesUpdateLen: ackRanges is null.\n");
        return ERROR;
    }
    if(ackRanges->ackRangesList.head == NULL) {
        DEBUG_PRINT("quicACKRangesUpdateLen: all ack ranges acked ready.\n");
        return SUCCESS;
    }
    // check which block is hit by the received packet
    uint16_t currOffset = 0; // the offset of the current block begin
    QUIC_ACK_Ranges_Block_t *currBlock = ackRanges->ackRangesList.head;
    QUIC_ACK_Ranges_Block_t *prevBlock = NULL;
    while (currBlock != NULL) {
        const uint16_t blockTotalLen = currBlock->gap + currBlock->ackRangeLength; // the number of packets contained in the entire block
        if (packetNumberOffset >= currOffset && packetNumberOffset < currOffset + blockTotalLen) { // the block found
            if (packetNumberOffset >= currOffset + currBlock->gap) {
                DEBUG_PRINT("quicACKRangesUpdateLen: the packet is already acked.\n");
                return SUCCESS;
            }
            if (packetNumberOffset == currOffset + currBlock->gap - 1) { // the packet is next to the len, in other words, the packet is in the end of the gap
                currBlock->gap--;
                currBlock->ackRangeLength++;
                if (currBlock->gap == 0) { // the gap is 0, we need to merge the block, all condition that gap = 0, len > 0, will be handled here
                    if (prevBlock != NULL) {
                        prevBlock->ackRangeLength += currBlock->ackRangeLength;
                        prevBlock->next = currBlock->next;
                        currBlock->next = NULL;
                        if (currBlock == ackRanges->ackRangesList.tail) ackRanges->ackRangesList.tail = prevBlock; // block is tail
                        // move the block to free list
                        quicACKRangesMoveToFree(ackRanges, currBlock);
                        ackRanges->ackRangesList.ackRangeCount--;
                    }
                    else { // the block is head
                        ackRanges->ackRangesList.head = (QUIC_ACK_Ranges_Block_t *)currBlock->next;
                        if (currBlock == ackRanges->ackRangesList.tail) ackRanges->ackRangesList.tail = NULL; // block is head and tail
                        // move the block to free list
                        quicACKRangesMoveToFree(ackRanges, currBlock);
                        ackRanges->ackRangesList.ackRangeCount--;
                        // return the acked len
                        return currBlock->ackRangeLength;
                    }
                }
            }
            else if (packetNumberOffset == currOffset) { // the packet is in the start of the block
                if (prevBlock != NULL) {
                    prevBlock->ackRangeLength += 1;
                    currBlock->gap -= 1;
                }
                else { // current block is head
                    currBlock->gap -= 1;
                    return 1; // return the acked len
                }
            }
            else { // the packet is in the middle of gap, we need to split this block
                const uint16_t firstBlockGap = packetNumberOffset - currOffset;
                const uint16_t firstBlockLen = 1;
                const uint16_t secondBlockGap = currBlock->gap - firstBlockGap - firstBlockLen;
                const uint16_t secondBlockLen = currBlock->ackRangeLength;
                // modify curr block to the first block
                currBlock->gap = firstBlockGap;
                currBlock->ackRangeLength = firstBlockLen;
                // create a new block for the second block
                QUIC_ACK_Ranges_Block_t *newBlock = quicACKRangesBlockGet(ackRanges);
                newBlock->gap = secondBlockGap;
                newBlock->ackRangeLength = secondBlockLen;
                // insert the new block to the list
                newBlock->next = currBlock->next;
                currBlock->next = (struct QUIC_ACK_Ranges_Block_t*)newBlock;
                if (currBlock == ackRanges->ackRangesList.tail) ackRanges->ackRangesList.tail = newBlock;
                ackRanges->ackRangesList.ackRangeCount++;
            }
            break;
        }
        currOffset += blockTotalLen;
        prevBlock = currBlock;
        currBlock = (QUIC_ACK_Ranges_Block_t*) currBlock->next;
    }
    // return
    if (currBlock == NULL) {
        DEBUG_PRINT("quicACKRangesUpdateLen: the packet is not in the ack ranges list.\n");
        return ERROR;
    }
    return SUCCESS;
}

static int quicACKRangesClear(QUIC_ACK_Ranges_t *ackRanges) {
    if(ackRanges == NULL) {
        DEBUG_PRINT("quicACKRangesClear: ackRanges is null.\n");
        return ERROR;
    }
    /* free ranges list */
    QUIC_ACK_Ranges_Block_t *currBlock = ackRanges->ackRangesList.head;
    while (currBlock != NULL) {
        QUIC_ACK_Ranges_Block_t *freeBlock = currBlock;
        currBlock = (QUIC_ACK_Ranges_Block_t*)currBlock->next;
        free(freeBlock);
    }
    ackRanges->ackRangesList.head = NULL;
    ackRanges->ackRangesList.tail = NULL;
    ackRanges->ackRangesList.ackRangeCount = 0;
    /* free the free blocks */
    QUIC_ACK_Ranges_Block_t *currFreeBlock = ackRanges->freeBlocks;
    while (currFreeBlock != NULL) {
        QUIC_ACK_Ranges_Block_t *freeBlock = currFreeBlock;
        currFreeBlock = (QUIC_ACK_Ranges_Block_t*)currFreeBlock->next;
        free(freeBlock);
    }
    /* return */
    return SUCCESS;
}

/* Stream map operations */
static void *quicStreamItemSet(Map_t *streamsMap, void *streamItem, const uint16_t streamId, const QUIC_NODE_STATUS status) {
    char mapKey[10] = {0};
    itoa(streamId, mapKey, 10);
    if (status == QUIC_CLIENT) {
        return mapSet(streamsMap, mapKey, streamItem, sizeof(QUIC_Send_Stream_Item_t));
    }
    if (status == QUIC_SERVER) {
        return mapSet(streamsMap, mapKey, streamItem, sizeof(QUIC_Read_Stream_Item_t));
    }
    return NULL;
}

static void readStreamItemRemoveCallback(Map_Node_t *node) {
    if (node == NULL) {
        DEBUG_PRINT("streamItemRemoveCallback: node is null.\n");
    }
    QUIC_Read_Stream_Item_t *readStreamItem = node->value;
    quicReadStreamClear(readStreamItem);
}

static void quicReadStreamItemRemove(Map_t *streamsMap, const uint16_t streamId) {
    char mapKey[10] = {0};
    itoa(streamId, mapKey, 10);
    mapRemove(streamsMap, mapKey, readStreamItemRemoveCallback); // TODO: may debug
}

static void sendStreamItemRemoveCallback(Map_Node_t *node) {
    if (node == NULL) {
        DEBUG_PRINT("streamItemRemoveCallback: node is null.\n");
    }
    QUIC_Send_Stream_Item_t *sendStreamItem = node->value;
    quicSendStreamClear(sendStreamItem);
}

static void quicSendStreamItemRemove(Map_t *streamsMap, const uint16_t streamId){
    char mapKey[10] = {0};
    itoa(streamId, mapKey, 10);
    mapRemove(streamsMap, mapKey, sendStreamItemRemoveCallback); // TODO: may debug
}

static void *quicStreamItemGet(Map_t *streamsMap, const uint16_t streamId) {
    char mapKey[10] = {0};
    itoa(streamId, mapKey, 10);
    return mapGet(streamsMap, mapKey);
}

static void quicStreamItemsMapInit(QUIC_Stream_t *streams, const QUIC_NODE_STATUS status) {
    Map_t *streamsMap = &streams->streamsMap;
    if (status == QUIC_CLIENT) {
        mapInit(streamsMap, MAP_TYPE_QUIC_SEND_STREAM, MAP_COPY_ADDR, QUIC_STREAM_NUMBER_MAX,
                sizeof(QUIC_Send_Stream_Item_t));
    }
    if (status == QUIC_SERVER) {
        mapInit(streamsMap, MAP_TYPE_QUIC_READ_STREAM, MAP_COPY_ADDR, QUIC_STREAM_NUMBER_MAX,
                sizeof(QUIC_Read_Stream_Item_t));
    }
    streams->streamItemSet = quicStreamItemSet;
    streams->streamItemGet = quicStreamItemGet;
}

/* connection state machine transport */
static int quicStateTransport(const uint16_t connId, const QUIC_NODE_STATUS status) {
    /* Transport state */
    if (status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *clientConnItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connId);
        const QUIC_CLIENT_CONN_STATE_TYPE state = clientConnItem->currentState;
        switch(state) {
            case QUIC_CLIENT_CONN_STATE_INITIAL:
                clientConnItem->currentState = QUIC_CLIENT_CONN_STATE_HELLO_SENT;
                break;
            case QUIC_CLIENT_CONN_STATE_HELLO_SENT:
                if (clientConnItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK == 1 && clientConnItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK == 1) {
                    clientConnItem->currentState = QUIC_CLIENT_CONN_STATE_PARAMETER_HANDLED;
                } else {
                    DEBUG_PRINT("In quicStateTransport: current client connection state is hello sent, but transport condition is not right.\n");
                }
                break;
            case QUIC_CLIENT_CONN_STATE_PARAMETER_HANDLED:
                clientConnItem->currentState = QUIC_CLIENT_CONN_STATE_ACKNOWLEDGE_SENT;
                break;
            case QUIC_CLIENT_CONN_STATE_ACKNOWLEDGE_SENT:
                if (clientConnItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK == 1 && clientConnItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK == 2) {
                    clientConnItem->currentState = QUIC_CLIENT_CONN_STATE_OPEN;
                    /* Transporting */
                    xTaskGenericNotify(clientConnItem->userTaskHandle, 0, clientConnItem->connId, eSetValueWithOverwrite, NULL);
                } else {
                    DEBUG_PRINT("In quicStateTransport: current client connection state is acknowledge sent, but transport condition is not right.\n");
                }
                break;
            case QUIC_CLIENT_CONN_STATE_OPEN:
                DEBUG_PRINT("In quicStateTransport: current client connection state is open.\n");
                break;
            default:
                break;
        }
    }
    if (status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *serverConnItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
        const QUIC_SERVER_CONN_STATE_TYPE state = serverConnItem->currentState;
        switch(state) {
            case QUIC_SERVER_CONN_STATE_INITIAL:
                if (serverConnItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK == 1) {
                    serverConnItem->currentState = QUIC_SERVER_CONN_STATE_HELLO_HANDLED;
                } else {
                    DEBUG_PRINT("In quicStateTransport: current server connection state is initial, but transport condition is not right.\n");
                }
                break;
            case QUIC_SERVER_CONN_STATE_HELLO_HANDLED:
                serverConnItem->currentState = QUIC_SERVER_CONN_STATE_PARAMETER_SENT;
                break;
            case QUIC_SERVER_CONN_STATE_PARAMETER_SENT:
                if (serverConnItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK == 2 && serverConnItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK == 1) {
                    serverConnItem->currentState = QUIC_SERVER_CONN_STATE_ACKNOWLEDGE_HANDLED;
                } else {
                    DEBUG_PRINT("In quicStateTransport: current server connection state is parameter sent, but transport condition is not right.\n");
                }
                break;
            case QUIC_SERVER_CONN_STATE_ACKNOWLEDGE_HANDLED:
                serverConnItem->currentState = QUIC_SERVER_CONN_STATE_OPEN;
                break;
            case QUIC_SERVER_CONN_STATE_OPEN:
                DEBUG_PRINT("In quicStateTransport: current server connection state is open.\n");
                break;
            default:
                break;
        }
    }
    return SUCCESS;
}

/* Connection map operations */
static void *quicConnItemSet(Map_t *connItemsMap, const uint16_t connId, void *connItem, const QUIC_NODE_STATUS status) {
    char mapKey[10] = {0};
    itoa(connId, mapKey, 10);
    if (status == QUIC_CLIENT) {
        return mapSet(connItemsMap, mapKey, connItem, sizeof(QUIC_Client_Conn_Item_t));
    }
    if (status == QUIC_SERVER) {
        return mapSet(connItemsMap, mapKey, connItem, sizeof(QUIC_Server_Conn_Item_t));
    }
    return NULL;
}

static void *quicConnItemGet(Map_t *connItemsMap, const uint16_t connId) {
    char mapKey[10] = {0};
    itoa(connId, mapKey, 10);
    return mapGet(connItemsMap, mapKey);
}

static void quicConnItemsMapInit(QUIC_Conn_t *conn, const QUIC_NODE_STATUS status) {
    Map_t *connItemMap = &conn->connItemsMap;
    if (status == QUIC_CLIENT) {
        mapInit(connItemMap, MAP_TYPE_QUIC_CLIENT_CONN, MAP_COPY_ADDR, QUIC_CONNECTION_NUMBER_MAX, sizeof(QUIC_Client_Conn_Item_t));
    }
    if (status == QUIC_SERVER) {
        mapInit(connItemMap, MAP_TYPE_QUIC_SERVER_CONN, MAP_COPY_ADDR, QUIC_CONNECTION_NUMBER_MAX, sizeof(QUIC_Server_Conn_Item_t));
    }
    conn->connItemSet = quicConnItemSet;
    conn->connItemGet = quicConnItemGet;
}

/* Timer callbacks */
static int gcd(int a, int b) {
    if (a == 0 || b == 0) {
        DEBUG_PRINT("gcd: a or b is 0.\n");
    }
    while(b != 0) {
        const int temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

void quicServerConnTimerCallback(TimerHandle_t xTimer) {
    QUIC_Server_Conn_Item_t *connItem = pvTimerGetTimerID(xTimer);
    /* calculate the appropriate clock cycle */
    connItem->timerPeriod = gcd(connItem->connTimeout, connItem->connACKPeriod);
    /* get current time */
    const TickType_t currentTime = xTaskGetTickCount();
    /* check whether the ack sending period has expired */
    if ((currentTime - connItem->lastACKTime) * 1000 / configTICK_RATE_HZ >= connItem->connACKPeriod) {
        /* send ack */
        quicServerSendACK(connItem->peer, connItem->connId);
        connItem->lastACKTime = currentTime;
    }
    /* check whether the connection times out */
    if ((currentTime - connItem->lastReceiveTime) * 1000 / configTICK_RATE_HZ >= connItem->connTimeout) {
        /* close connection */
        quicServerConnClose(connItem->connId);
    }
}

static void retransmitPacket(void *packetInfoNode) {
    QUIC_Packet_Info_Node_t *packetInfo = packetInfoNode;
    /* get current time */
    const TickType_t currentTime = xTaskGetTickCount();
    /* check whether the retransmission period has expired */
    if ((currentTime - packetInfo->sendTime) * 1000 / configTICK_RATE_HZ >= packetInfo->retransmitPeriod) {
        /* retransmit packet */
        quicClientResendData(packetInfo);
        packetInfo->retransmitCount++;
        packetInfo->sendTime = currentTime;
    }
}

void quicClientConnTimerCallback(TimerHandle_t xTimer) {
    QUIC_Client_Conn_Item_t *connItem = pvTimerGetTimerID(xTimer);
    /* calculate the appropriate clock cycle */
    connItem->timerPeriod = gcd(connItem->connTimeout, connItem->retransmitPeriod);
    /* get current time */
    TickType_t currentTime = xTaskGetTickCount();
    /* check whether the retransmission period has expired */
    traverseRBTree(connItem->packetInfoManager.packetInfoRBTree, retransmitPacket);
    /* check whether the connection times out */
    currentTime = xTaskGetTickCount();
    if ((currentTime - connItem->lastReceiveTime) * 1000 / configTICK_RATE_HZ >= connItem->connTimeout) {
        /* close connection */
        quicClientConnClose(connItem->connId);
    }
}

/* Tx task */
static void quicTxTask() {
    systemWaitStart();

    int pos = 0;
    while(true) {
        if(quicClientNode.isOpen) {
            for (; pos < QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT; pos++) {
                if (quicClientNode.transportInfo[pos].connectionID != 0) {
                    quicClientSendData(quicClientNode.transportInfo[pos].connectionID,
                                       quicClientNode.transportInfo[pos].streamID); // the stream is running, then get data from buffer, and send it
                    break;
                }
            }
            pos++;
            if (pos >= QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT) pos = 0;
        }
        vTaskDelay(M2T(1));
    }
}

/* Rx task */
static void quicRxTask() {
    systemWaitStart();

    UWB_Data_Packet_t dataRxPacket;
    while(true) {
        if(uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_QUIC, &dataRxPacket)) {
            if(!quicServerNode.isOpen) {
                // TODO: send message to client, server is not open
                vTaskDelay(M2T(1));
                continue;
            }
            /* Get receive timestamp, we don't really need precise timing */
            const TickType_t receiveTime = xTaskGetTickCount();
            /* We may receive packet as client or server */
            DEBUG_PRINT("Received QUIC packet from %u\n", dataRxPacket.header.srcAddress);
            const UWB_Address_t peer = dataRxPacket.header.srcAddress;
            const uint16_t payloadLen = dataRxPacket.header.length - sizeof(UWB_Data_Packet_Header_t);
            uint16_t curPosLen = 0;
            uint8_t peerStatus = 0;
            uint16_t peerDstConnId = 0;
            bool error = false;
            while(curPosLen < payloadLen) {
                const uint8_t headerForm = dataRxPacket.payload[curPosLen] >> 7;
                if (headerForm != QUIC_LONG_HEADER && headerForm != QUIC_SHORT_HEADER) {
                    DEBUG_PRINT("Error in quicRxTask, header form is not exist, packet analyse error, packet abandoned.\n");
                    break;
                }
                int packetOffset = 0;
                if(headerForm == QUIC_LONG_HEADER) {
                    Quic_Long_Packet_t *packet = (Quic_Long_Packet_t *) &dataRxPacket.payload[curPosLen];
                    const uint8_t type = packet->header.longPacketType;
                    peerStatus = packet->header.status;
                    peerDstConnId = packet->header.dstConnId;
                    switch(type) {
                        case QUIC_INITIAL_PACKET:
                            DEBUG_PRINT("In quicRxTask: Initial packet received.\n");
                            packetOffset = quicProcessInitialPacket(packet, peer, receiveTime);
                            break;
                        case QUIC_ZERO_RTT_PACKET:
                            break;
                        case QUIC_HANDSHAKE_PACKET:
                            DEBUG_PRINT("In quicRxTask: Handshake packet received.\n");
                            packetOffset = quicProcessHandshakePacket(packet, peer, receiveTime);
                            break;
                        default:
                            DEBUG_PRINT("Error in quicRxTask, long packet type is not exist.\n");
                            break;
                    }
                }
                if (headerForm == QUIC_SHORT_HEADER){
                     Quic_One_RTT_Packet_t *packet = (Quic_One_RTT_Packet_t *) &dataRxPacket.payload[curPosLen];
                     packetOffset = quicProcessOneRTTPacket(packet, receiveTime);
                }
                error = packetOffset == ERROR;
                if (error) break;
                curPosLen += packetOffset;
            }
            /* State transport */
            if(peerStatus == QUIC_CLIENT && !error && peerDstConnId == 0) error = quicStateTransport(quicTempSrcConnId, QUIC_SERVER);
            else if(peerStatus == QUIC_CLIENT && !error && peerDstConnId != 0) error = quicStateTransport(peerDstConnId, QUIC_SERVER);
            else if(peerStatus == QUIC_SERVER && !error && peerDstConnId != 0) error = quicStateTransport(peerDstConnId, QUIC_CLIENT);
#ifdef QUIC_TEST_SERVER
            /* Send integrated packets to peer node */
            if(peerStatus == QUIC_CLIENT && !error) {
                const QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, peerDstConnId);
                if (connItem == NULL) connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, quicTempSrcConnId); /* When server receive connection request message */
                if (connItem == NULL) {
                    DEBUG_PRINT("Error in server quicRxTask, connection item is not exist.\n");
                    break;
                }
                ASSERT(connItem != NULL); /* If connection is still null, then connection is not exist */
                switch(connItem->currentState) {
                    case QUIC_SERVER_CONN_STATE_HELLO_HANDLED:
                        quicServerSendConnReply(peer, quicTempSrcConnId);
                        break;
                    case QUIC_SERVER_CONN_STATE_ACKNOWLEDGE_HANDLED:
                        quicServerSendConnDone(peer, peerDstConnId);
                        break;
                    case QUIC_SERVER_CONN_STATE_OPEN:
                        // TODO: Send 1-RTT packet
                        DEBUG_PRINT("In quicRxTask: Send 1-RTT packet.\n");
                        break;
                    default:
                        break;
                }
            } else if(peerStatus == QUIC_SERVER && !error) {
                const QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, peerDstConnId);
                if (connItem == NULL) {
                    DEBUG_PRINT("Error in client quicRxTask, connection item is not exist.\n");
                    break;
                }
                ASSERT(connItem != NULL);
                switch(connItem->currentState) {
                    case QUIC_CLIENT_CONN_STATE_PARAMETER_HANDLED:
                        quicClientSendConnReply(peer, connItem->connId);
                        break;
                    case QUIC_CLIENT_CONN_STATE_OPEN:
                        // TODO: Send 1-RTT packet
                        DEBUG_PRINT("In quicRxTask: Send 1-RTT packet.\n");
                        break;
                    default:
                        break;
                }
            }
#endif
        }
        vTaskDelay(M2T(1));
    }
}

void quicInit() {
    rxPacketQueue = xQueueCreate(QUIC_RX_PACKET_QUEUE_SIZE, QUIC_RX_PACKET_ITEM_SIZE);
    streamNotifyQueue = xQueueCreate(QUIC_STREAM_NOTIFY_QUEUE_SIZE, QUIC_STREAM_NOTIFY_QUEUE_ITEM_SIZE);

    quicConnIdMutex = xSemaphoreCreateMutex();
    quicStreamIdMutex = xSemaphoreCreateMutex();
    UWB_Data_Packet_Listener_t listener = {
        .type = UWB_DATA_MESSAGE_QUIC,
        .rxQueue = rxPacketQueue
    };
    uwbRegisterDataPacketListener(&listener);

    // If I receive a packet from server, or send a packet to server, then I am client
    quicClientNode.me = uwbGetAddress();
    quicClientNode.mu = xSemaphoreCreateMutex();
    quicClientNode.conns.size = 0;
    quicClientNode.conns.capacity = QUIC_CONNECTION_NUMBER_MAX;
    quicClientNode.isOpen = false;
    quicConnItemsMapInit(&quicClientNode.conns, QUIC_CLIENT);
    // If I receive a packet from client, or send a packet to client, then I am server
    quicServerNode.me = uwbGetAddress();
    quicServerNode.mu = xSemaphoreCreateMutex();
    quicServerNode.conns.size = 0;
    quicServerNode.conns.capacity = QUIC_CONNECTION_NUMBER_MAX;
    quicServerNode.isOpen = false;
    quicConnItemsMapInit(&quicServerNode.conns, QUIC_SERVER);

    xTaskCreate(quicRxTask, ADHOC_DECK_QUIC_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL, ADHOC_DECK_TASK_PRI, &quicRxTaskHandle);
    xTaskCreate(quicTxTask, ADHOC_DECK_QUIC_TX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL, ADHOC_DECK_TASK_PRI, &quicTxTaskHandle);
}

/* Frame Operations */
int quicGenerateTypeFrame(Quic_Long_Packet_t *packet, const uint16_t framePos, const QUIC_FRAME_TYPE type) {
    if(type >= QUIC_FRAME_PARAMETER) {
        DEBUG_PRINT("Error in generate only type frame, type is not exist.");
        return ERROR;
    }
    if(packet->header.length + sizeof(Quic_Type_Frame_t) >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add frame, payload overflow.");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add frame, payload overflow.");
        return ERROR;
    }
    Quic_Type_Frame_t *frame = (Quic_Type_Frame_t *) &packet->packetPayload[framePos];
    memset(frame, 0, sizeof(Quic_Type_Frame_t));
    frame->type = type;
    ASSERT(packet->header.length + sizeof(Quic_Type_Frame_t) < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    packet->header.length += sizeof(Quic_Type_Frame_t);
    // /* If handshake done frame */
    // if(type == QUIC_FRAME_HANDSHAKE_DONE) {
    //     QUIC_Server_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.srcConnId);
    //     ASSERT(connItem != NULL);
    // }
    return sizeof(Quic_Type_Frame_t);
}

int quicHandleHelloFrame(const Quic_Long_Packet_t *packet, const UWB_Address_t peer) {
    ASSERT(packet->header.status == QUIC_CLIENT);
    /* receive duplicated connection request */
    if (quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, packet->header.dstConnId) != NULL) {
        DEBUG_PRINT("Error in quicHandleHelloFrame, duplicated connection request.");
        return ERROR;
    }
    /* Create new connection in server */
    QUIC_Server_Conn_Item_t connItem_ = {0};
    connItem_.peer = peer;
    connItem_.connId = getNextSrcConnId();
    connItem_.currentState = QUIC_SERVER_CONN_STATE_INITIAL;
    connItem_.dstConnId = packet->header.srcConnId;
    // connItem_.minimumFinishedStreamId = 0;
    connItem_.freeBlockPoolHead = dataBlockInit(0);
    connItem_.connACKPeriod = QUIC_ACK_DELAY_MAX_DEFAULT;
    connItem_.connTimeout = QUIC_MAX_IDLE_TIMEOUT_DEFAULT;
    connItem_.lastACKTime = xTaskGetTickCount();
    connItem_.lastReceiveTime = xTaskGetTickCount();
    connItem_.timerPeriod = QUIC_SERVER_CONN_TIMER_PERIOD_DEFAULT;
    QUIC_Client_Conn_Item_t *connItem = quicServerNode.conns.connItemSet(&quicServerNode.conns.connItemsMap, connItem_.connId, &connItem_, QUIC_SERVER);
    quicACKRangesInit(&connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges, QUIC_ACK_RANGES_CAPACITY_DEFAULT);
    connItem->timer = xTimerCreate("quicServerConnTimer", pdMS_TO_TICKS(QUIC_SERVER_CONN_TIMER_PERIOD_DEFAULT), pdTRUE, (void *)connItem, quicServerConnTimerCallback);
    if (connItem->timer == NULL) {
        DEBUG_PRINT("quicHandleHelloFrame: create timer failed.\n");
        return ERROR;
    }
    xTimerStart(connItem->timer, 0);
    quicStreamItemsMapInit(&connItem->sendStreams, QUIC_SERVER);
    quicServerNode.conns.size++;

    quicTempSrcConnId = connItem->connId; /* Will be used when server reply */
    /* Calculate length and return */
    return sizeof(Quic_Type_Frame_t);
}

int quicHandleHandshakeDoneFrame(const Quic_Long_Packet_t *packet, int pos, UWB_Address_t peer) {
    ASSERT(packet->header.status == QUIC_SERVER);
    DEBUG_PRINT("quicHandleHandshakeDoneFrame: Handshake done frame received.\n");
    // QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
    // ASSERT(connItem != NULL);
    // connItem->stateTransport(connItem, QUIC_CLIENT);
    return sizeof(Quic_Type_Frame_t);
}

int quicGenerateACKFrame(Quic_Long_Packet_t *packet, const int framePos, const void *connItem_) {
    if (connItem_ == NULL) {
        DEBUG_PRINT("Error in generate ACK frame, connection item is not exist.");
        return ERROR;
    }
    /* Get connection states and write ACK frame*/
    Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) &packet->packetPayload[framePos];
    memset(frame, 0, sizeof(Quic_ACK_Frame_Header_t));
    frame->header.type = QUIC_FRAME_ACK;
    /* Select ACK packet type */
    QUIC_PACKET_ACK_TYPE ackType = 0;
    switch (packet->header.longPacketType) {
        case QUIC_INITIAL_PACKET:
            ackType = QUIC_INITIAL_PACKET_ACK;
            break;
        case QUIC_HANDSHAKE_PACKET:
            ackType = QUIC_HANDSHAKE_PACKET_ACK;
            break;
        case QUIC_ZERO_RTT_PACKET:
            ackType = QUIC_ZERO_RTT_PACKET_ACK;
            break;
        default:
            break;
    }
    /* Generate ACK frame */
    if(packet->header.status == QUIC_CLIENT) {
        const QUIC_Client_Conn_Item_t *connItem = connItem_;
        ASSERT(connItem != NULL);
        frame->header.largestACK = connItem->packetReceiveWindow[ackType].largestACK;
        frame->header.ACKDelay = packet->header.headerForm == QUIC_LONG_HEADER ? 0 : QUIC_ACK_DELAY_MAX_DEFAULT; // TODO: set timer and calculate delay;
        frame->header.firstACKRange = connItem->packetReceiveWindow[ackType].largestACK - connItem->packetReceiveWindow[ackType].minimumACK;
        // TODO: set ACK ranges when flow control is implemented
        for(int i = 0; i < frame->header.ACKRangeCount; i++) {
            frame->ACKRange[i].gap = 0;
            frame->ACKRange[i].ackRangeLength = 0;
        }
    }
    if (packet->header.status == QUIC_SERVER) {
        const QUIC_Server_Conn_Item_t *connItem = connItem_;
        ASSERT(connItem != NULL);
        frame->header.largestACK = connItem->packetReceiveWindow[ackType].largestACK;
        frame->header.ACKDelay = packet->header.headerForm == QUIC_LONG_HEADER ? 0 : QUIC_ACK_DELAY_MAX_DEFAULT; // TODO: set timer and calculate delay;
        frame->header.firstACKRange = connItem->packetReceiveWindow[ackType].largestACK - connItem->packetReceiveWindow[ackType].minimumACK;
        // TODO: set ACK ranges when flow control is implemented
        for(int i = 0; i < frame->header.ACKRangeCount; i++) {
            frame->ACKRange[i].gap = 0;
            frame->ACKRange[i].ackRangeLength = 0;
        }
    }
    packet->header.length += sizeof(Quic_ACK_Frame_Header_t) + frame->header.ACKRangeCount * sizeof(Quic_ACK_Range_t);
    if(packet->header.length >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add ACK frame, payload overflow.\n");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add ACK frame, payload overflow.\n");
        return ERROR;
    }
    ASSERT(packet->header.length < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    // DEBUG_PRINT("In quicGenerateACKFrame: largestACK: %u, ACKDelay: %u, ACKRangeCount: %u, firstACKRange: %u.\n", frame->header.largestACK, frame->header.ACKDelay, frame->header.ACKRangeCount, frame->header.firstACKRange);
    return (int)sizeof(Quic_ACK_Frame_Header_t) + (int)(frame->header.ACKRangeCount * sizeof(Quic_ACK_Range_t));
}

int quicHandleACKFrame(const Quic_Long_Packet_t *packet, const int pos, UWB_Address_t peer) {
    const int packetType = packet->header.longPacketType;
    int windowType = 0;
    switch(packetType) {
        case QUIC_INITIAL_PACKET:
            windowType = QUIC_INITIAL_PACKET_ACK;
            break;
        case QUIC_ZERO_RTT_PACKET:
            windowType = QUIC_ZERO_RTT_PACKET_ACK;
            break;
        case QUIC_HANDSHAKE_PACKET:
            windowType = QUIC_HANDSHAKE_PACKET_ACK;
            break;
        default:
            DEBUG_PRINT("Error in quicHandleACKFrame, packet type is not exist.\n");
            return ERROR;
    }
    const Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) &packet->packetPayload[pos];
    if(packet->header.status == QUIC_SERVER) {
        const QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("Error in quicHandleACKFrame, connection item is not exist.\n");
            return ERROR;
        }
        ASSERT(connItem != NULL);
        if(connItem->packetSendWindow[windowType].largestACK != frame->header.largestACK) {
            // TODO: retransmission last packet
            DEBUG_PRINT("In quicHandleACKFrame: retransmission last packet.\n");
        }
    } else if(packet->header.status == QUIC_CLIENT) {
        const QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, packet->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("Error in quicHandleACKFrame, connection item is not exist.\n");
            return ERROR;
        }
        ASSERT(connItem != NULL);
        if(connItem->packetSendWindow[windowType].largestACK != frame->header.largestACK) {
            // TODO: retransmission last packet
            DEBUG_PRINT("In quicHandleACKFrame: retransmission last packet.\n");
        }
    }

    return (int) sizeof(Quic_ACK_Frame_Header_t) + frame->header.ACKRangeCount * (int) sizeof(Quic_ACK_Range_t);
}

int quicGenerateParameterFrame(Quic_Long_Packet_t *packet, const uint16_t framePos) {
    if(packet->header.length + sizeof(Quic_Parameter_Frame_t) >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        if(packet->header.longPacketType == QUIC_INITIAL_PACKET) DEBUG_PRINT("Error in Initial packet add parameter frame, payload overflow.");
        else if(packet->header.longPacketType == QUIC_HANDSHAKE_PACKET) DEBUG_PRINT("Error in handshake packet add parameter frame, payload overflow.");
        return ERROR;
    }
    Quic_Parameter_Frame_t *frame = (Quic_Parameter_Frame_t *) &packet->packetPayload[framePos];
    frame->type = QUIC_FRAME_PARAMETER;
    frame->size = 0;
    /* Add parameters */
    int parameterPos = 0;
    const int padding = 2 * sizeof(uint8_t);
    /* Max Idle Timeout */
    Quic_Parameter_Frame_Item_t *parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_IDLE_TIMEOUT;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_MAX_IDLE_TIMEOUT_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Max routing payload size */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_ROUTE_PAYLOAD_SIZE;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_MAX_ROUTE_PAYLOAD_SIZE_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max data */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_DATA;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_INITIAL_MAX_DATA_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max stream data uni */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_STREAM_DATA_UNI;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Initial max streams uni */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_INITIAL_MAX_STREAMS_UNI;
    parameter->parameterLength = sizeof(uint8_t);
    *(uint8_t *)&parameter->parameterValue = QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* ACK delay exponent */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_ACK_DELAY_EXPONENT;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_ACK_DELAY_EXPONENT_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Max ACK delay */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_MAX_ACK_DELAY;
    parameter->parameterLength = sizeof(uint16_t);
    *(uint16_t *)&parameter->parameterValue = QUIC_ACK_DELAY_MAX_DEFAULT;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;
    /* Active connection ID limit */
    parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
    parameter->parameterID = QUIC_ACTIVE_CONNECTION_ID_LIMIT;
    parameter->parameterLength = sizeof(uint8_t);
    *(uint8_t *)&parameter->parameterValue = QUIC_CONNECTION_NUMBER_MAX;
    parameterPos += padding + parameter->parameterLength;
    frame->size++;

    ASSERT(packet->header.length + 2 * sizeof(uint8_t) + parameterPos < QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX);
    packet->header.length += 2 * sizeof(uint8_t) + parameterPos;
    return parameterPos + 2 * (int)sizeof(uint8_t);
}

int quicHandleParameterFrame(Quic_Long_Packet_t *packet, const int pos, UWB_Address_t peer) {
    if(packet->header.status != QUIC_SERVER) {
        DEBUG_PRINT("Error in quicHandleParameterFrame, peer status not right.\n");
        return ERROR;
    }
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
    Quic_Parameter_Frame_t *frame = (Quic_Parameter_Frame_t *) &packet->packetPayload[pos];
    int parameterPos = 0;
    const int padding = 2 * sizeof(uint8_t);
    for(int i = 0; i < frame->size; i++) {
        const Quic_Parameter_Frame_Item_t *parameter = (Quic_Parameter_Frame_Item_t *) &frame->parameters[parameterPos];
        // DEBUG_PRINT("In quicHandleParameterFrame: parameter ID: %u, parameter length: %u, \n", parameter->parameterID, parameter->parameterLength);
        switch(parameter->parameterID) {
            case QUIC_MAX_IDLE_TIMEOUT:
                connItem->transportParamsTuple.parameters[QUIC_MAX_IDLE_TIMEOUT] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_MAX_IDLE_TIMEOUT]);
                break;
            case QUIC_MAX_ROUTE_PAYLOAD_SIZE:
                connItem->transportParamsTuple.parameters[QUIC_MAX_ROUTE_PAYLOAD_SIZE] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_MAX_ROUTE_PAYLOAD_SIZE]);
                break;
            case QUIC_INITIAL_MAX_DATA:
                connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_DATA] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_DATA]);
                break;
            case QUIC_INITIAL_MAX_STREAM_DATA_UNI:
                connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_STREAM_DATA_UNI] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_STREAM_DATA_UNI]);
                break;
            case QUIC_INITIAL_MAX_STREAMS_UNI:
                connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_STREAMS_UNI] = *(uint8_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_INITIAL_MAX_STREAMS_UNI]);
                break;
            case QUIC_ACK_DELAY_EXPONENT:
                connItem->transportParamsTuple.parameters[QUIC_ACK_DELAY_EXPONENT] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_ACK_DELAY_EXPONENT]);
                break;
            case QUIC_MAX_ACK_DELAY:
                connItem->transportParamsTuple.parameters[QUIC_MAX_ACK_DELAY] = *(uint16_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_MAX_ACK_DELAY]);
                break;
            case QUIC_ACTIVE_CONNECTION_ID_LIMIT:
                connItem->transportParamsTuple.parameters[QUIC_ACTIVE_CONNECTION_ID_LIMIT] = *(uint8_t *)&parameter->parameterValue;
                // DEBUG_PRINT("parameter value: %u.\n", connItem->transportParamsTuple.parameters[QUIC_ACTIVE_CONNECTION_ID_LIMIT]);
                break;
            default:
                DEBUG_PRINT("Error in quicHandleParameterFrame, parameter ID is not exist.\n");
                return ERROR;
        }
        parameterPos += padding + parameter->parameterLength;
    }

    connItem->transportParamsTuple.size = frame->size;

    return padding + parameterPos;
}

/* Generate stream frame, buffer data is filled to the packet according to the remaining space of the packet， offset==0 is used to determine whether to use an extension header */
int quicGenerateStreamFrame(Quic_One_RTT_Packet_t *packet, const uint16_t framePos, const uint16_t connID, const uint16_t streamID, const uint32_t restLength) {
    if(packet->header.status != QUIC_CLIENT) {
        DEBUG_PRINT("Error in quicGenerateStreamFrame, peer status is not client.\n");
    }
    if(packet->header.length + sizeof(Quic_Stream_Frame_Header_t) > QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX) {
        DEBUG_PRINT("Error in quicGenerateStreamFrame, payload overflow.\n");
    }
    /* Get stream buffer */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connID);
    QUIC_Send_Stream_Item_t *streamItem = connItem->sendStreams.streamItemGet(&connItem->sendStreams.streamsMap, streamID);
    /* determine whether to use extensions */
    bool isExtension = false;
    if (streamItem->dataBuffer.sendOffset == 0) {
        isExtension = true;
    }
    /* rest length */
    uint32_t payloadLength = 0;
    if (isExtension) payloadLength = restLength - sizeof(Quic_Stream_Frame_Extend_Header_t);
    else payloadLength = restLength - sizeof(Quic_Stream_Frame_Header_t);
    /* read send buffer */
    QUIC_Stream_Sending_Data_t sendingData = {0};
    streamItem->readSendBuffer(&streamItem->dataBuffer, &sendingData, payloadLength, packet->header.packetNumber);
    /* Generate frame header */
    if (isExtension) {
        Quic_Stream_Extend_Frame_t *frame = (Quic_Stream_Extend_Frame_t *) &packet->packetPayload[framePos];
        memset(frame, 0, sizeof(Quic_Stream_Frame_Extend_Header_t));
        frame->extendHeader.originHeader.type = sendingData.isFin ? QUIC_FRAME_STREAM_EXTEND_AND_FIN : QUIC_FRAME_STREAM_EXTEND; /* stream extend */
        frame->extendHeader.originHeader.streamID = streamID;
        frame->extendHeader.originHeader.offset = sendingData.offset;
        frame->extendHeader.originHeader.length = sizeof(Quic_Stream_Frame_Extend_Header_t) + payloadLength;
        frame->extendHeader.prevStreamID = streamItem->prevStreamID;
        frame->extendHeader.nextStreamID = streamItem->nextStreamID;
        /* write payload */
        memcpy(&frame->data[0], sendingData.data, payloadLength);
    } else {
        Quic_Stream_Frame_t *frame = (Quic_Stream_Frame_t *) &packet->packetPayload[framePos];
        memset(frame, 0, sizeof(Quic_Stream_Frame_Header_t));
        frame->header.type = sendingData.isFin ? QUIC_FRAME_STREAM_FIN : QUIC_FRAME_STREAM; /* stream or stream fin */
        frame->header.streamID = streamID;
        frame->header.offset = sendingData.offset;
        frame->header.length = sizeof(Quic_Stream_Frame_Header_t) + payloadLength;
        /* Write payload */
        memcpy(&frame->data[0], sendingData.data, payloadLength);
    }
    /* send stream state machine change */
    if (sendingData.isFin && streamItem->sendingStatus == QUIC_STREAM_SENDING_SEND) {
        streamItem->sendingStatus = QUIC_STREAM_SENDING_DATA_SENT;
        quicTransformInfoDelete(connItem->connId, streamItem->streamId); // because after this time, we only need to resend lost data with timer, no active sending is required
    }

    return (int)sizeof(Quic_Stream_Frame_Header_t) + (int)payloadLength;
}

int quicHandleStreamFrame(const Quic_One_RTT_Packet_t *packet, const int pos) {
    if(packet->header.status != QUIC_SERVER) {
        DEBUG_PRINT("Error in quicHandleStreamFrame, peer status is not server.\n");
    }
    const uint8_t type = packet->packetPayload[pos];
    const Quic_Stream_Frame_t *frame = (Quic_Stream_Frame_t *) &packet->packetPayload[pos];
    /* Get stream from connection item */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, packet->header.dstConnId);
    QUIC_Read_Stream_Item_t *streamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, frame->header.streamID);
    /* check if it is the stream's last data frame */
    bool isFin = false;
    if (type == QUIC_FRAME_STREAM_FIN || type == QUIC_FRAME_STREAM_EXTEND_AND_FIN) {
        isFin = true;
    }
    /* check if it is extended stream frame */
    bool isExtension = false;
    if (type == QUIC_FRAME_STREAM_EXTEND || type == QUIC_FRAME_STREAM_EXTEND_AND_FIN) {
        isExtension = true;
    }
    /* check if the stream is existed */
    if(streamItem == NULL) { // this is a new stream, while this frame is the first frame of the stream
        QUIC_Read_Stream_Item_t streamItem_ = {0};
        streamItem_.streamId = frame->header.streamID;
        streamItem_.streamGroupId = 0; // means not head stream, and it has head stream
        streamItem_.isHeadStream = false;
        streamItem_.prevStreamID = 0;
        streamItem_.nextStreamID = 0;
        streamItem_.dataBuffer.receiveBlockList.head = NULL;
        streamItem_.dataBuffer.receiveBlockList.tail = NULL;
        streamItem_.dataBuffer.receiveBlockList.count = 0;
        streamItem_.dataBuffer.readOffset = 0;
        streamItem_.dataBuffer.consumedOffset = 0;
//        streamItem_.dataBuffer.maxReceiveOffset = QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT;
        streamItem_.dataBuffer.freeBlocksHead = connItem->freeBlockPoolHead;
        streamItem_.dataBuffer.receivedFin = false;
        streamItem_.dataBuffer.isIntegrity = false;
        streamItem_.connItemPtr = (struct QUIC_Server_Conn_Item_t *)connItem;
        // if (frame->header.type == QUIC_FRAME_STREAM_HEAD || frame->header.type == QUIC_FRAME_STREAM_HEAD_FIN) { // if the stream is head stream, then we set some segment
        //     streamItem_.isHeadStream = true;
        //     streamItem_.streamGroupId = getNextStreamGroupId();
        // }
        if (isExtension) { // check extend
            const Quic_Stream_Extend_Frame_t *extendFrame = (Quic_Stream_Extend_Frame_t *) &packet->packetPayload[pos];
            streamItem_.prevStreamID = extendFrame->extendHeader.prevStreamID;
            streamItem_.nextStreamID = extendFrame->extendHeader.nextStreamID;
            if (extendFrame->extendHeader.prevStreamID == 0 && streamItem_.isHeadStream == false) { // check stream head
                streamItem_.isHeadStream = true;
                streamItem_.streamGroupId = getNextStreamGroupId();
            }
        }
        // if (!streamItem_.isHeadStream) { // if the stream is not head stream, then we check the previous stream is existed or not, and set the pointers
        //     streamItem_.preStreamID = frame->header.preStreamID;
        //     QUIC_Read_Stream_Item_t *prevStreamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, streamItem_.preStreamID);
        //     if (prevStreamItem != NULL) {
        //         // streamItem_.dataBuffer.prevDataBuffer = (struct QUIC_Stream_Receive_Buffer_t *)&prevStreamItem->dataBuffer; // we connect these buffers to facilitate future reads (quicReceiveStreamRead)
        //         // prevStreamItem->dataBuffer.nextDataBuffer = (struct QUIC_Stream_Receive_Buffer_t *)&streamItem_.dataBuffer;
        //         streamItem_.prevStreamItem = (struct QUIC_Read_Stream_Item_t*)prevStreamItem;
        //         prevStreamItem->nextStreamItem = (struct QUIC_Read_Stream_Item_t*)&streamItem_;
        //     }
        // } else {
        //     streamItem_.preStreamID = 0;
        //     // streamItem_.dataBuffer.nextDataBuffer = NULL;
        //     // streamItem_.dataBuffer.prevDataBuffer = NULL;
        //     streamItem_.prevStreamItem = NULL;
        //     streamItem_.nextStreamItem = NULL;
        // }
        /* link front related stream, if it does not link and is not head */
        if (streamItem_.prevStreamItem == NULL && streamItem_.prevStreamID != 0 && !streamItem_.isHeadStream) {
            QUIC_Read_Stream_Item_t *prevStreamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, streamItem_.prevStreamID);
            if (prevStreamItem != NULL) {
                streamItem_.prevStreamItem = (struct QUIC_Read_Stream_Item_t*)prevStreamItem;
                prevStreamItem->nextStreamItem = (struct QUIC_Read_Stream_Item_t*)&streamItem_;
            }
        }
        /* link front related stream, if it does not link yet */
        if (streamItem_.nextStreamItem == NULL && streamItem_.nextStreamID != 0) {
            QUIC_Read_Stream_Item_t *nextStreamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, streamItem_.nextStreamID);
            if (nextStreamItem != NULL) {
                streamItem_.nextStreamItem = (struct QUIC_Read_Stream_Item_t*)nextStreamItem;
                nextStreamItem->prevStreamItem = (struct QUIC_Read_Stream_Item_t*)&streamItem_;
            }
        }
        /* stream set */
        streamItem = connItem->readStreams.streamItemSet(&connItem->readStreams.streamsMap, &streamItem_, frame->header.streamID, QUIC_SERVER);
        streamItem->readReceiveBuffer = readStreamReceiveBuffer;
        streamItem->writeReceiveBuffer = writeStreamReceiveBuffer;
        streamItem->receivingStatus = QUIC_STREAM_RECEIVING_RECEIVE;
        if(streamItem == NULL) {
            DEBUG_PRINT("Error in quicHandleStreamFrame, create new stream item failed, space is not enough.\n");
            return ERROR;
        }
        assert(streamItem != NULL);
    } else if (!streamItem->isHeadStream && streamItem->prevStreamID > 0 && streamItem->prevStreamItem == NULL) { // because previous stream may not be created yet (packet has not received yet), so we need to check, and set pointers
        QUIC_Read_Stream_Item_t *prevStreamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, streamItem->prevStreamID);
        if (prevStreamItem != NULL) {
            // streamItem->dataBuffer.prevDataBuffer = (struct QUIC_Stream_Receive_Buffer_t *)&prevStreamItem->dataBuffer;
            // prevStreamItem->dataBuffer.nextDataBuffer = (struct QUIC_Stream_Receive_Buffer_t *)&streamItem->dataBuffer;
            streamItem->prevStreamItem = (struct QUIC_Read_Stream_Item_t*)prevStreamItem;
            prevStreamItem->nextStreamItem = (struct QUIC_Read_Stream_Item_t*)streamItem;
        }
    }
    /* write the received data to the buffer */
    streamItem->writeReceiveBuffer(&streamItem->dataBuffer, &frame->data[0], frame->header.length - sizeof(Quic_Stream_Frame_Header_t), frame->header.offset, isFin);
    /* stream's states machine check */
    if (streamItem->receivingStatus == QUIC_STREAM_RECEIVING_SIZE_KNOWN) { // the stream may receive last data frame, but packet loss may have occurred before, so we need to check every time receive a frame
        /* check data integrity and notify the user layer if it is complete */
        if(verifyStreamDataIntegrity(&streamItem->dataBuffer)) { // when the stream's data is integrity, we notify the transport layer
            streamItem->receivingStatus = QUIC_STREAM_RECEIVING_DATA_RECEIVED;
            streamItem->dataBuffer.isIntegrity = true;
            /* check, if the stream is head or not, | Stream Head | <--> | Stream | <--> | Stream | -no ptr- | Stream | */
            if (streamItem->isHeadStream) { // because we need to prove the data's sequence in a stream group
                /* notify transport layer, stream data is ready */
                QUIC_Transport_Info_t transportInfo = {0};
                transportInfo.peer = connItem->peer;
                transportInfo.connectionID = connItem->connId;
                transportInfo.streamID = streamItem->streamId;
                transportInfo.streamGroupID = streamItem->streamGroupId;
                xQueueSend(streamNotifyQueue, &transportInfo, 0);
            } else { // this stream is not head stream, so we need to check the previous stream is head or not
                QUIC_Read_Stream_Item_t *prevStreamItem = NULL;
                do {
                    prevStreamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, streamItem->prevStreamID);
                } while(prevStreamItem != NULL && !prevStreamItem->isHeadStream);
                if(prevStreamItem != NULL && prevStreamItem->dataBuffer.isIntegrity) {
                    QUIC_Transport_Info_t transportInfo = {0};
                    transportInfo.peer = connItem->peer;
                    transportInfo.connectionID = connItem->connId;
                    transportInfo.streamID = prevStreamItem->streamId;
                    transportInfo.streamGroupID = prevStreamItem->streamGroupId;
                    xQueueSend(streamNotifyQueue, &transportInfo, 0);
                }
            }
        }
    }
    if (isFin) { // if the stream receive final data frame, we need to set up the state machine and immediately check if the data is complete, and if it is complete, submit it
        streamItem->dataBuffer.receivedFin = true;
        streamItem->receivingStatus = QUIC_STREAM_RECEIVING_SIZE_KNOWN;
        /* check data integrity and notify the user layer if it is complete */
        if(verifyStreamDataIntegrity(&streamItem->dataBuffer)) {
            streamItem->receivingStatus = QUIC_STREAM_RECEIVING_DATA_RECEIVED;
            streamItem->dataBuffer.isIntegrity = true;
            /* check, if the stream is head or not, | Stream Head | <--> | Stream | <--> | Stream | -no ptr- | Stream | */
            if (streamItem->isHeadStream) { // because we need to prove the data sequence
                /* notify transport layer, stream data is ready */
                QUIC_Transport_Info_t transportInfo = {0};
                transportInfo.peer = connItem->peer;
                transportInfo.connectionID = connItem->connId;
                transportInfo.streamID = streamItem->streamId;
                transportInfo.streamGroupID = streamItem->streamGroupId;
                xQueueSend(streamNotifyQueue, &transportInfo, 0);
            } else {
                QUIC_Read_Stream_Item_t *prevStreamItem = NULL;
                do {
                    prevStreamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, streamItem->prevStreamID);
                } while(prevStreamItem != NULL && !prevStreamItem->isHeadStream);
                if(prevStreamItem != NULL && prevStreamItem->dataBuffer.isIntegrity) {
                    QUIC_Transport_Info_t transportInfo = {0};
                    transportInfo.peer = connItem->peer;
                    transportInfo.connectionID = connItem->connId;
                    transportInfo.streamID = prevStreamItem->streamId;
                    transportInfo.streamGroupID = prevStreamItem->streamGroupId;
                    xQueueSend(streamNotifyQueue, &transportInfo, 0);
                }
            }
        }
    }

    return (int)frame->header.length;
}

/* Only server need generate ack frame, ack timer will use this function */
int quicGenerateOneRTTPacketACKFrame(Quic_One_RTT_Packet_t *packet, const uint16_t framePos, const QUIC_Server_Conn_Item_t *connItem) {
    if (connItem == NULL) {
        DEBUG_PRINT("quicGenerateOneRTTPacketACKFrame: connection item is not exist.");
        return ERROR;
    }
    if(packet->header.length + sizeof(Quic_ACK_Frame_t) >= QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        DEBUG_PRINT("quicGenerateOneRTTPacketACKFrame: payload overflow.");
        return ERROR;
    }
    if (connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges == NULL) {
        DEBUG_PRINT("quicGenerateOneRTTPacketACKFrame: ack ranges is not exist.");
        return ERROR;
    }
    /* Generate ACK frame */
    Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) &packet->packetPayload[framePos];
    memset(frame, 0, sizeof(Quic_ACK_Frame_Header_t));
    frame->header.type = QUIC_FRAME_ACK;
    frame->header.largestACK = connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK;
    frame->header.ACKDelay = QUIC_ACK_DELAY_MAX_DEFAULT;
    frame->header.ACKRangeCount = connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges->ackRangesList.ackRangeCount <= QUIC_ACK_FRAME_RANGE_SIZE_MAX ? connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges->ackRangesList.ackRangeCount : QUIC_ACK_FRAME_RANGE_SIZE_MAX;
    if (connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges->ackRangesList.tail != NULL) frame->header.firstACKRange = connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges->ackRangesList.tail->ackRangeLength;
    else frame->header.firstACKRange = 0;
    frame->header.minimumACK = connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].minimumACK;
    /* write ack ranges */
    const QUIC_ACK_Ranges_Block_t *currBlock = connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges->ackRangesList.head;
    for (int i = 0; i < frame->header.ACKRangeCount; i++) {
        if (currBlock != NULL) {
            frame->ACKRange[i].gap = currBlock->gap;
            frame->ACKRange[i].ackRangeLength = currBlock->ackRangeLength;
            currBlock = (QUIC_ACK_Ranges_Block_t*)currBlock->next;
        } else {
            break;
        }
    }
    packet->header.length += sizeof(Quic_ACK_Frame_Header_t) + frame->header.ACKRangeCount * sizeof(Quic_ACK_Range_t);
    if (packet->header.length - sizeof(Quic_Short_Packet_Header_t) >= QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX) {
        DEBUG_PRINT("quicGenerateOneRTTPacketACKFrame: payload overflow.");
        return ERROR;
    }

    return (int) sizeof(Quic_ACK_Frame_Header_t) + frame->header.ACKRangeCount * (int) sizeof(Quic_ACK_Range_t);
}

/* Only client will use this function.
 * 1. Make all packet that number less than minimum acked packet number segment in ack frame be acked.
 * 2. Make all packet that acked (in 'len') in the ranges be acked locally.
 */
int quicHandleOneRTTPacketACKFrame(const Quic_One_RTT_Packet_t *packet, const int pos) {
    /* get ack frame */
    const Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) &packet->packetPayload[pos];
    const int minAckedPacketNumber = frame->header.minimumACK; // minimum acked packet number
    /* get client connection item */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
    /* make all packet that number less than minAckedPacketNumber be acked */
    while (connItem->packetInfoManager.minimumUnackedPacketNumber <= minAckedPacketNumber) { // packet number == minimum ack number, will be acked
        connItem->packetInfoManager.packetInfoNodeDelete(connItem->packetInfoManager.packetInfoRBTree, connItem->packetInfoManager.minimumUnackedPacketNumber, connItem->connId);
        connItem->packetInfoManager.minimumUnackedPacketNumber++;
    }
    /* ack packets that server has received */
    uint32_t toAckedPacketNumber = connItem->packetInfoManager.minimumUnackedPacketNumber;
    for (int i = 0; i < frame->header.ACKRangeCount; i++) { // start with (minimum ack number + 1)
        int gap = frame->ACKRange[i].gap;
        int len = frame->ACKRange[i].ackRangeLength;
        toAckedPacketNumber += gap; // position is the start of 'len'
        for(int j = 0; j < len; j++) { // handle every packet need to acked in 'len'
            connItem->packetInfoManager.packetInfoNodeDelete(connItem->packetInfoManager.packetInfoRBTree, toAckedPacketNumber + j, connItem->connId);
        }
        toAckedPacketNumber += len; // position is the start of next range part
    }

    return SUCCESS;
}

int quicGenerateResendStreamFrame(Quic_One_RTT_Packet_t *packet, uint16_t framePos, QUIC_Packet_Info_Node_t *packetInfo) {
    if(packet->header.length + sizeof(Quic_Stream_Frame_Header_t) > QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX) {
        DEBUG_PRINT("quicGenerateResendStreamFrame: payload overflow.\n");
    }
    /* determine whether to use extensions */
    bool isExtension = false;
    if (packetInfo->offset == 0) {
        isExtension = true;
    }
    /* Generate frame header */
    if (isExtension) {
        Quic_Stream_Extend_Frame_t *frame = (Quic_Stream_Extend_Frame_t *) &packet->packetPayload[framePos];
        memset(frame, 0, sizeof(Quic_Stream_Frame_Extend_Header_t));
        frame->extendHeader.originHeader.type = QUIC_FRAME_STREAM_EXTEND; /* stream extend */
        frame->extendHeader.originHeader.streamID = packetInfo->streamId;
        frame->extendHeader.originHeader.offset = packetInfo->offset;
        frame->extendHeader.originHeader.length = sizeof(Quic_Stream_Frame_Extend_Header_t) + packetInfo->length;
        /* get connection item */
        QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packetInfo->connId);\
        /* get stream item */
        QUIC_Send_Stream_Item_t *streamItem = connItem->sendStreams.streamItemGet(&connItem->sendStreams.streamsMap, packetInfo->streamId);

        frame->extendHeader.prevStreamID = streamItem->prevStreamID;
        frame->extendHeader.nextStreamID = streamItem->nextStreamID;
        /* write payload */
        memcpy(&frame->data[0], packetInfo->dataBlock->data, packetInfo->length);
    } else {
        Quic_Stream_Frame_t *frame = (Quic_Stream_Frame_t *) &packet->packetPayload[framePos];
        memset(frame, 0, sizeof(Quic_Stream_Frame_Header_t));
        frame->header.type = QUIC_FRAME_STREAM; /* stream */
        frame->header.streamID = packetInfo->streamId;
        frame->header.offset = packetInfo->offset;
        frame->header.length = sizeof(Quic_Stream_Frame_Header_t) + packetInfo->length;
        /* Write payload */
        memcpy(&frame->data[0], packetInfo->dataBlock->data, packetInfo->length);
    }

    return (int)sizeof(Quic_Stream_Frame_Header_t) + (int)packetInfo->length;
}

/* Packet Operations */
int quicGenerateInitialPacket(Quic_Long_Packet_t *packet, const uint16_t srcConnId, const uint16_t dstConnId, void *connItem, const QUIC_NODE_STATUS status) {
    memset(packet, 0, sizeof(Quic_Long_Packet_Header_t));
    packet->header.headerForm = 1;
    packet->header.fixedBit = 1;
    packet->header.longPacketType = QUIC_INITIAL_PACKET;
    packet->header.srcConnId = srcConnId;
    packet->header.dstConnId = dstConnId;
    packet->header.status = status;

    /* Initial packet send ACK handle and packet number handle */
    if(packet->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *clientConnItem = connItem;
        clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].largestACK++;
        clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].minimumACK++;
        clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = clientConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].largestACK;
    } else if(packet->header.status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *serverConnItem = connItem;
        serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].largestACK++;
        serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].minimumACK++;
        serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = serverConnItem->packetSendWindow[QUIC_INITIAL_PACKET_ACK].largestACK;
    }
    
    packet->header.length = sizeof(Quic_Long_Packet_Header_t);

    return sizeof(Quic_Long_Packet_Header_t);
}

int quicProcessInitialPacket(const Quic_Long_Packet_t *initialPacket, const UWB_Address_t peer, const TickType_t receiveTime) {
    const uint16_t payloadLen = initialPacket->header.length - sizeof(Quic_Long_Packet_Header_t);
    uint16_t curPosLen = 0;
    while(curPosLen < payloadLen) {
        const uint8_t type = initialPacket->packetPayload[curPosLen];
        int frameOffset = 0;
        switch(type) {
        case QUIC_FRAME_HELLO:
            DEBUG_PRINT("In quicProcessInitialPacket: handle Hello frame.\n");
            frameOffset = quicHandleHelloFrame(initialPacket, peer);
            break;
        case QUIC_FRAME_ACK:
            DEBUG_PRINT("In quicProcessInitialPacket: handle ack frame.\n");
            frameOffset = quicHandleACKFrame(initialPacket, curPosLen, peer);
            break;
        default:
            DEBUG_PRINT("Error in process initial packet, frame type is not exist.\n");
            return ERROR;
        }
        if (frameOffset == ERROR) {
            return ERROR;
        }
        curPosLen += frameOffset;
    }

    /* Initial packet receive ACK handle and ConnID handle */
    if(initialPacket->header.status == QUIC_CLIENT) {
        const uint16_t connId = initialPacket->header.dstConnId == 0 ? quicTempSrcConnId : initialPacket->header.dstConnId;
        QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
        connItem->lastReceiveTime = receiveTime; // update last receive time
        /* ConnID Handle */
        connItem->dstConnId = initialPacket->header.srcConnId;
        /* Receive ACK */
        // MARK
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].minimumACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].ackRanges = NULL;
    } else if(initialPacket->header.status == QUIC_SERVER) {
        QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, initialPacket->header.dstConnId);
        connItem->lastReceiveTime = receiveTime; // update last receive time
        /* ConnID Handle */
        connItem->dstConnId = initialPacket->header.srcConnId;
        /* Receive ACK */
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].largestACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].minimumACK = initialPacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_INITIAL_PACKET_ACK].ackRanges = NULL;
    }
    
    return (int) initialPacket->header.length;
}

int quicGenerateHandshakePacket(Quic_Long_Packet_t *packet, const uint16_t srcConnId, const uint16_t dstConnId, void *connItem, const QUIC_NODE_STATUS status) {
    memset(packet, 0, sizeof(Quic_Long_Packet_Header_t));
    packet->header.headerForm = 1;
    packet->header.fixedBit = 1;
    packet->header.longPacketType = QUIC_HANDSHAKE_PACKET;
    packet->header.srcConnId = srcConnId;
    packet->header.dstConnId = dstConnId;
    packet->header.status = status;

    /* Handshake packet send ACK handle and packet number handle */
    if(packet->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *clientConnItem = connItem;
        clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK++;
        clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].minimumACK++;
        clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = clientConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK;
    } else if(packet->header.status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *serverConnItem = connItem;
        serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK++;
        serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].minimumACK++;
        serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = serverConnItem->packetSendWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK;
    }
    
    packet->header.length = sizeof(Quic_Long_Packet_Header_t);

    return sizeof(Quic_Long_Packet_Header_t);
}

int quicProcessHandshakePacket(Quic_Long_Packet_t *handshakePacket, const UWB_Address_t peer, TickType_t receiveTime) {
    const uint16_t payloadLen = handshakePacket->header.length - sizeof(Quic_Long_Packet_Header_t);
    uint16_t curPosLen = 0;
    while(curPosLen < payloadLen) {
        const uint8_t type = handshakePacket->packetPayload[curPosLen];
        int frameOffset = 0;
        switch(type) {
        case QUIC_FRAME_ACK:
            DEBUG_PRINT("In quicProcessHandshakePacket: handle ack frame.\n");
            frameOffset = quicHandleACKFrame(handshakePacket, curPosLen, peer);
            break;
        case QUIC_FRAME_PARAMETER:
            DEBUG_PRINT("In quicProcessHandshakePacket: handle parameter frame.\n");
            frameOffset = quicHandleParameterFrame(handshakePacket, curPosLen, peer);
            break;
        case QUIC_FRAME_HANDSHAKE_DONE:
            DEBUG_PRINT("In quicProcessHandshakePacket: handle handshake done frame.\n");
            frameOffset = quicHandleHandshakeDoneFrame(handshakePacket, curPosLen, peer);
            break;
        default:
            DEBUG_PRINT("Error in process handshake packet, frame type is not exist.\n");
            return ERROR;
        }
        if (frameOffset == ERROR) {
            return ERROR;
        }
        curPosLen += frameOffset;
    }

    /* Handshake packet receive ACK handle and ConnID handle */
    if(handshakePacket->header.status == QUIC_CLIENT) {
        QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, handshakePacket->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("Error in process handshake packet, connection item is not exist.\n");
            return ERROR;
        }
        connItem->lastReceiveTime = receiveTime; // update last receive time
        /* Receive ACK */
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK = handshakePacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].minimumACK = handshakePacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRanges = NULL;
    } else if(handshakePacket->header.status == QUIC_SERVER) {
        QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, handshakePacket->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("Error in process handshake packet, connection item is not exist.\n");
            return ERROR;
        }
        connItem->lastReceiveTime = receiveTime; // update last receive time
        /* ConnID Handle */
        connItem->dstConnId = handshakePacket->header.srcConnId;
        /* Receive ACK */
        ASSERT(connItem != NULL);
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].largestACK = handshakePacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].minimumACK = handshakePacket->header.packetNumber;
        connItem->packetReceiveWindow[QUIC_HANDSHAKE_PACKET_ACK].ackRanges = NULL;
    }

    return (int) handshakePacket->header.length;
}

/* Enter the header of the 1-RTT packet and automatically process the sliding window */
int quicGenerateOneRTTPacket(Quic_One_RTT_Packet_t *packet, const uint16_t dstConnId, const QUIC_NODE_STATUS status) {
    /* Generate packet header */
    memset(packet, 0, sizeof(Quic_Short_Packet_Header_t));
    packet->header.headerForm = QUIC_SHORT_HEADER;
    packet->header.fixedBit = 1;
    packet->header.dstConnId = dstConnId;
    packet->header.status = status;
    packet->header.length = sizeof(Quic_Short_Packet_Header_t);
    packet->header.status = status;

    /* 1-RTT packet send packet number handle */
    if (packet->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("quicGenerateOneRTTPacket: client connection item is not exist.\n");
            return ERROR;
        }
        connItem->packetInfoManager.largestPacketNumber++;
        packet->header.packetNumber = connItem->packetInfoManager.largestPacketNumber;
    } else if (packet->header.status == QUIC_SERVER) {
        QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("quicGenerateOneRTTPacket: server connection item is not exist.\n");
            return ERROR;
        }
        connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK++;
        connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].minimumACK++;
        connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges = NULL;
        packet->header.packetNumber = connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK;
    }

    /* The ACK(Packet info) handle will be set after generate stream frame */

    packet->header.length = sizeof(Quic_Short_Packet_Header_t);

    return SUCCESS;
}

int quicProcessOneRTTPacket(const Quic_One_RTT_Packet_t *oneRTTPacket, TickType_t receiveTime) {
    const uint16_t payloadLen = oneRTTPacket->header.length - sizeof(Quic_Short_Packet_Header_t);
    uint16_t curPosLen = 0;
    while (curPosLen < payloadLen) {
        const uint8_t type = oneRTTPacket->packetPayload[curPosLen];
        int frameOffset = 0;
        switch (type) {
        case QUIC_FRAME_STREAM:
            DEBUG_PRINT("quicProcessOneRTTPacket: handle stream frame.\n");
            // TODO: I need to think about how to modify quicHandleStreamFrame to make it judge whether the stream is fin or not
            frameOffset = quicHandleStreamFrame(oneRTTPacket, curPosLen);
            break;
        case QUIC_FRAME_STREAM_FIN:
            DEBUG_PRINT("quicProcessOneRTTPacket: handle stream fin frame.\n");
            frameOffset = quicHandleStreamFrame(oneRTTPacket, curPosLen);
            break;
        case QUIC_FRAME_STREAM_EXTEND:
            DEBUG_PRINT("quicProcessOneRTTPacket: handle head stream frame.\n");
            frameOffset = quicHandleStreamFrame(oneRTTPacket, curPosLen);
            break;
        case QUIC_FRAME_STREAM_EXTEND_AND_FIN:
            DEBUG_PRINT("quicProcessOneRTTPacket: handle head stream fin frame.\n");
            frameOffset = quicHandleStreamFrame(oneRTTPacket, curPosLen);
            break;
        case QUIC_FRAME_ACK:
            DEBUG_PRINT("quicProcessOneRTTPacket: handle ack frame.\n");
            // Only client need it
            frameOffset = quicHandleOneRTTPacketACKFrame(oneRTTPacket, curPosLen);
            break;
        default:
            DEBUG_PRINT("quicProcessOneRTTPacket: Error in process handshake packet, frame type is not exist.\n");
            return ERROR;
        }
        if (frameOffset == ERROR) return ERROR;
        curPosLen += frameOffset;
    }
    /* Receive ack handle */
    if (oneRTTPacket->header.status == QUIC_CLIENT) {
        QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, oneRTTPacket->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("quicProcessOneRTTPacket: server connection item is not exist.\n");
            return ERROR;
        }
        connItem->lastReceiveTime = receiveTime; // update last receive time
        if (oneRTTPacket->header.packetNumber > connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK) { // when packet number > largestACK, that means packet between largestACK and packet number (largestACK, packetNumber) are lost
            quicACKRangesAddGap(connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges, oneRTTPacket->header.packetNumber - connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK - 1);
            connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK = oneRTTPacket->header.packetNumber;
        }
        else if (oneRTTPacket->header.packetNumber == connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK) { // packet number == largestACK, no need to handle
            DEBUG_PRINT("quicProcessOneRTTPacket: packet number is equal to the largestACK, no need to handle.\n");
        }
        else { // packet number < largestACK, that means the packet should be acked
            const int res = quicACKRangesUpdateLen(connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges, oneRTTPacket->header.packetNumber - connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].minimumACK - 1);
            if (res == ERROR) {
                DEBUG_PRINT("quicProcessOneRTTPacket: Error in update ACK ranges.\n");
                return ERROR;
            }
            connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].minimumACK += res;
        }
    } else if (oneRTTPacket->header.status == QUIC_SERVER) {
        QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, oneRTTPacket->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("quicProcessOneRTTPacket: client connection item is not exist.\n");
            return ERROR;
        }
        connItem->lastReceiveTime = receiveTime; // update last receive time
    }

    return SUCCESS;
}

/* Message Operations */
int quicClientSendConnRequest(const UWB_Address_t peer, TaskHandle_t userTaskHandle) {
    /* Steps:
     * 1. Generate initial packet
     * 2. Generate Hello frame
     * 3. Send packet
     */
    /* Generate data packet */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);

    const int srcConnId = getNextSrcConnId();
    const int dstConnId = 0;
    /* Initialize local connection */
    /* Because we issued a connection request, we need to be prepared to maintain the connection. */
    if(quicClientNode.conns.size + 1 > quicClientNode.conns.capacity) {
        DEBUG_PRINT("Connection is full, can not resolve this connection.\n");
        return ERROR;
    }
    QUIC_Client_Conn_Item_t connItem_ = {0};
    connItem_.connId = srcConnId;
    connItem_.dstConnId = dstConnId;
    connItem_.currentState = QUIC_CLIENT_CONN_STATE_INITIAL;
    connItem_.peer = peer;
    connItem_.userTaskHandle = userTaskHandle;
    connItem_.freeBlockPoolHead = dataBlockInit(0);
    /* since memset, packetTuples are already set 0. */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemSet(&quicClientNode.conns.connItemsMap, srcConnId, &connItem_, QUIC_CLIENT);
    quicPacketInfoManagerInit(&connItem->packetInfoManager);
    connItem->timer = xTimerCreate("quicClientConnTimer", pdMS_TO_TICKS(QUIC_CLIENT_CONN_TIMER_PERIOD_DEFAULT), pdTRUE, (void *)connItem, quicClientConnTimerCallback); // create connection timer
    if (connItem->timer == NULL) {
        DEBUG_PRINT("quicHandleHelloFrame: create timer failed.\n");
        return ERROR;
    }
    xTimerStart(connItem->timer, 0);
    quicStreamItemsMapInit(&connItem->sendStreams, QUIC_CLIENT);
    quicClientNode.conns.size++;
    /* Generate packets */
    int packetPos = 0;
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateInitialPacket(initialPacket, srcConnId, dstConnId, connItem, QUIC_CLIENT);
    /* Generate frames */
    int framePos = 0;
    /* Generate Hello frame */
    framePos += quicGenerateTypeFrame(initialPacket, framePos, QUIC_FRAME_HELLO);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicClientSendConnRequest: initial packet send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(srcConnId, QUIC_CLIENT);

    return SUCCESS;
}

int quicServerSendConnReply(const UWB_Address_t peer, const uint16_t connId) {
    /* Steps:
     * 1. Generate initial packet
     * 2. Generate ack frame
     * 3. Generate handshake packet
     * 4. Generate parameter frame
     * 5. Send packet
     */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicServerNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
    if (connItem == NULL) return ERROR;
    /* Generate packets */
    int packetPos = 0;
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateInitialPacket(initialPacket, connItem->connId, connItem->dstConnId, connItem, QUIC_SERVER);
    /* Generate frames */
    int framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(initialPacket, framePos, connItem);
    packetPos += framePos;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, connItem, QUIC_SERVER);
    /* Generate frames */
    framePos = 0;
    /* Generate parameter frame */
    framePos += quicGenerateParameterFrame(handshakePacket, framePos);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicServerSendConnReply: initial and handshake packets send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(connId, QUIC_SERVER);

    return SUCCESS;
}

int quicClientSendConnReply(const UWB_Address_t peer, const uint16_t connId) {
    /* Steps:
     * 1. Generate initial packet
     * 2. Generate ack frame
     * 3. Generate handshake packet
     * 4. Generate ack frame
     * 5. Send packet
     */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate initial packet */
    Quic_Long_Packet_t *initialPacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateInitialPacket(initialPacket, connItem->connId, connItem->dstConnId, connItem, QUIC_CLIENT);
    /* Generate frames */
    int framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(initialPacket, framePos, connItem);
    packetPos += framePos;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, connItem, QUIC_CLIENT);
    /* Generate frames */
    framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(handshakePacket, framePos, connItem);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicClientSendConnReply: initial, handshake and 1-RTT packets send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(connId, QUIC_CLIENT);

    return SUCCESS;
}

int quicServerSendConnDone(const UWB_Address_t peer, const uint16_t connId) {
    /* Steps:
     * 1. Generate handshake packet
     * 2. Generate handshake done frame
     * 3. Generate ack frame
     * 4. Send packet
     */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicServerNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate handshake packet */
    Quic_Long_Packet_t *handshakePacket = (Quic_Long_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos += quicGenerateHandshakePacket(handshakePacket, connItem->connId, connItem->dstConnId, connItem, QUIC_SERVER);
    /* Generate frames */
    int framePos = 0;
    /* Generate handshake done frame */
    framePos += quicGenerateTypeFrame(handshakePacket, framePos, QUIC_FRAME_HANDSHAKE_DONE);
    /* Generate ACK frame */
    framePos += quicGenerateACKFrame(handshakePacket, framePos, connItem);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicServerSendConnDone: handshake and 1-RTT packets send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(connId, QUIC_SERVER);

    return SUCCESS;
}

int quicServerSendACK(UWB_Address_t peer, uint16_t connId) {
    /*
     * Steps:
     * 1. Generate 1-RTT packet
     * 2. Generate 1-RTT ACK frame
     * 3. Send packet
     */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicServerNode.me;
    dataTxPacket.header.destAddress = peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Find connection with peer */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
    /* Generate packets */
    int packetPos = 0;
    /* Generate 1-RTT packet */
    Quic_One_RTT_Packet_t *oneRTTPacket = (Quic_One_RTT_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos = quicGenerateOneRTTPacket(oneRTTPacket, connItem->dstConnId, QUIC_SERVER);
    /* Generate frames */
    int framePos = 0;
    /* Generate ACK frame */
    framePos += quicGenerateOneRTTPacketACKFrame(oneRTTPacket, framePos, connItem);
    packetPos += framePos;

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("quicServerSendACK: send ACK packet.\n");
    uwbSendDataPacketBlock(&dataTxPacket);

    return SUCCESS;
}

/* connection close */
static void quicReadStreamClearCallback(const Map_Node_t *node) {
    if (node == NULL) return;
    QUIC_Read_Stream_Item_t *streamItem = node->value;
    if (streamItem == NULL) return;
    /* clear the stream */
    quicReadStreamClear(streamItem);
}

static void quicServerConnClearCallback(const Map_Node_t *node) {
    if (node == NULL) return;
    QUIC_Server_Conn_Item_t *connItem = node->value;
    if (connItem == NULL) return;
    /* free the connection's space */
    free(connItem);
}

int quicServerConnClose(const uint16_t connId) {
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connId);
    if (connItem == NULL) {
        DEBUG_PRINT("quicServerConnClose: connection item is not exist.\n");
        return ERROR;
    }
    /* clear all streams in the connection (include streams' buffer) */
    mapClear(&connItem->readStreams.streamsMap, quicReadStreamClearCallback);
    /* clear ack windows' ack ranges */
    quicACKRangesClear(connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges);
    /* free the free block pool */
    DataBlock_t *currBlock = connItem->freeBlockPoolHead;
    while (currBlock != NULL) {
        DataBlock_t *nextBlock = currBlock->next;
        dataBlockClear(currBlock);
        currBlock = nextBlock;
    }
    /* remove connection item */
    char mapKey[10] = {0};
    itoa(connId, &mapKey[0], 10);
    mapRemove(&quicServerNode.conns.connItemsMap, &mapKey[0], quicServerConnClearCallback);
    /* return */
    return SUCCESS;
}

static void quicSendStreamClearCallback(const Map_Node_t *node) {
    if (node == NULL) return;
    QUIC_Send_Stream_Item_t *streamItem = node->value;
    if (streamItem == NULL) return;
    /* clear the stream */
    quicSendStreamClear(streamItem);
}

static void quicClientConnClearCallback(const Map_Node_t *node) {
    if (node == NULL) return;
    QUIC_Client_Conn_Item_t *connItem = node->value;
    if (connItem == NULL) return;
    /* free the connection's space */
    free(connItem);
}

int quicClientConnClose(uint16_t connId) {
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connId);
    if (connItem == NULL) {
        DEBUG_PRINT("quicClientConnClose: connection item is not exist.\n");
        return ERROR;
    }
    /* clear all streams in the connection (include streams' buffer) */
    mapClear(&connItem->sendStreams.streamsMap, quicSendStreamClearCallback);
    /* clear the RBTree */
    clearRBTree(connItem->packetInfoManager.packetInfoRBTree);
    /* free the free block pool */
    DataBlock_t *currBlock = connItem->freeBlockPoolHead;
    while (currBlock != NULL) {
        DataBlock_t *nextBlock = currBlock->next;
        dataBlockClear(currBlock);
        currBlock = nextBlock;
    }
    /* remove connection item */
    char mapKey[10] = {0};
    itoa(connId, &mapKey[0], 10);
    mapRemove(&quicClientNode.conns.connItemsMap, &mapKey[0], quicClientConnClearCallback);
    /* return */
    return SUCCESS;
}

/* Stream operations */
/* Client send stream data, if stream not exists, then create and send data */
int quicClientSendData(const uint16_t connID, const uint16_t streamID) {
    /* Steps:
     * 1. Get connection item
     * 2. Prepare data packet
     * 3. Generate 1-RTT packet
     * 4. Generate stream frame
     * 5. Send packet
     */
    /* get connection item first */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connID);
    if (connItem == NULL) {
        DEBUG_PRINT("Error in quicClientSendData, connection item is not exist.");
        return ERROR;
    }
    const QUIC_Send_Stream_Item_t *streamItem = connItem->sendStreams.streamItemGet(&connItem->sendStreams.streamsMap, streamID);
    if (streamItem == NULL) { // before send data, stream item and data have been created anyway
        DEBUG_PRINT("Error in quicClientSendData, stream item is not exist.");
        return ERROR;
    }
    /* if now stream's state is not send, return */
    if (streamItem->sendingStatus != QUIC_STREAM_SENDING_SEND) {
        DEBUG_PRINT("Error in quicClientSendData, stream is not ready to send data.");
        return ERROR;
    }
    /* prepare data packet */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = connItem->peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Generate packets */
    int packetPos = 0;
    /* Generate 1-RTT packet header */
    Quic_One_RTT_Packet_t *oneRTTPacket = (Quic_One_RTT_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos = quicGenerateOneRTTPacket(oneRTTPacket, connItem->dstConnId, QUIC_CLIENT);
    /* Generate frames */
    int framePos = 0;
    /* Generate stream frame */
    /* write data to stream frame */
    const uint32_t restLength = QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX - sizeof(Quic_Short_Packet_Header_t);
    framePos += quicGenerateStreamFrame(oneRTTPacket, framePos, connItem->connId, streamID, restLength);

    packetPos += framePos;
    dataTxPacket.header.length += packetPos;
    /* handle ack(packet info) */
    QUIC_Packet_Info_Node_t packetInfoNode = {0};
    quicPacketInfoNodeInit(&packetInfoNode, oneRTTPacket, streamItem);
    QUIC_Packet_Info_Manager_t *packetInfoManager = &connItem->packetInfoManager;
    packetInfoManager->packetInfoNodeInsert(packetInfoManager->packetInfoRBTree, &packetInfoNode);
    /* Send packet */
    DEBUG_PRINT("quicClientSendData: send data packet.\n");
    uwbSendDataPacketBlock(&dataTxPacket);
    /* return */
    return SUCCESS;
}

/*
 * Create a new stream in the connection.
 * @param connID: connection ID
 * @return: stream ID
 */
int quicSendStreamCreate(uint16_t connID, uint16_t streamID, uint16_t prevStreamID, uint16_t nextStreamID) {
    /* get connection item first */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connID);
    if (connItem == NULL) {
        DEBUG_PRINT("Error in quicSendStreamCreate, connection item is not exist.");
        return ERROR;
    }
    /* check if over largest stream number */
    if (connItem->sendStreams.size + 1 >= QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT) {
        DEBUG_PRINT("Error in quicSendStreamCreate, stream number is over the largest stream number.");
        return ERROR;
    }
    /* create a new stream */
    QUIC_Send_Stream_Item_t streamItem_ = {0};
    streamItem_.streamId = streamID;
    streamItem_.dataBuffer.pendingBlockList.head = NULL;
    streamItem_.dataBuffer.pendingBlockList.tail = NULL;
    streamItem_.dataBuffer.pendingBlockList.count = 0;
    streamItem_.dataBuffer.unackedBlockList.head = NULL;
    streamItem_.dataBuffer.unackedBlockList.tail = NULL;
    streamItem_.dataBuffer.unackedBlockList.count = 0;
    streamItem_.dataBuffer.freeBlocksHead = connItem->freeBlockPoolHead;
    streamItem_.dataBuffer.maxSendOffset = 0;
    streamItem_.dataBuffer.sendOffset = 0;
    streamItem_.dataBuffer.sentFin = false;
    streamItem_.writeSendBuffer = writeStreamSendBuffer;
    streamItem_.readSendBuffer = readStreamSendBuffer;
    if (prevStreamID == 0) streamItem_.isHeadStream = true;
    else streamItem_.isHeadStream = false;
    streamItem_.prevStreamID = prevStreamID;
    streamItem_.nextStreamID = nextStreamID;
    streamItem_.connItemPtr = (struct QUIC_Client_Conn_Item_t*)connItem;
    QUIC_Send_Stream_Item_t *streamItem = connItem->sendStreams.streamItemSet(&connItem->sendStreams.streamsMap, &streamItem_, streamID, QUIC_CLIENT);
    streamItem->sendingStatus = QUIC_STREAM_SENDING_READY;
    if (streamItem == NULL) {
        DEBUG_PRINT("Error in quicSendStreamCreate, create new stream item failed, space is not enough.");
        return ERROR;
    }
    connItem->sendStreams.size++;

    return SUCCESS;
}

/*
 *  write data to the stream buffer, slice data into suitable size, and write it to buffer block
 *  @param connID: connection ID
 *  @param streamID: stream ID
 *  @param data: data to write
 *  @param len: data length
 *  @param isLastSegment: is the last segment, if it is, then in lower layer will set fin flag
 *  @return: SUCCESS or ERROR
 */
int quicSendStreamWrite(uint16_t connID, uint16_t streamID, const uint8_t *data, const uint32_t len, bool isLastSegment) {
    /* get connection item first */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connID);
    if (connItem == NULL) {
        DEBUG_PRINT("Error in quicSendStreamWrite, connection item is not exist.");
        return ERROR;
    }
    /* get stream item */
    QUIC_Send_Stream_Item_t *streamItem = connItem->sendStreams.streamItemGet(&connItem->sendStreams.streamsMap, streamID);
    if (streamItem == NULL) {
        DEBUG_PRINT("Error in quicSendStreamWrite, stream item is not exist.");
        return ERROR;
    }
    /* write data to stream buffer, slice data into suitable size, and write it to buffer block */
    uint32_t restLen = len;
    uint32_t curPos = 0;
    while (restLen > 0) {
        uint32_t writeLen = restLen > QUIC_STREAM_DATA_BLOCK_MAX_DATA_SIZE ? QUIC_STREAM_DATA_BLOCK_MAX_DATA_SIZE : restLen; // slice data into suitable size, the size is fixed by QUIC_STREAM_DATA_BLOCK_SIZE
        xSemaphoreTake(quicClientNode.mu, portMAX_DELAY);
        if (restLen == writeLen && isLastSegment == true) { // judge if it is the last block, if it is, then set fin flag
            if (streamItem->writeSendBuffer(&streamItem->dataBuffer, data + curPos, writeLen, true) == ERROR) {
                DEBUG_PRINT("Error in quicStreamWrite, write data to stream buffer failed.");
                xSemaphoreGive(quicClientNode.mu);
                return ERROR;
            }
            xSemaphoreGive(quicClientNode.mu);
        } else { // else just write data to buffer
            if (streamItem->writeSendBuffer(&streamItem->dataBuffer, data + curPos, writeLen, false) == ERROR) {
                DEBUG_PRINT("Error in quicStreamWrite, write data to stream buffer failed.");
                xSemaphoreGive(quicClientNode.mu);
                return ERROR;
            }
        }
        xSemaphoreGive(quicClientNode.mu);

        restLen -= writeLen;
        curPos += writeLen;
    }
    /* send stream state machine change */
    streamItem->sendingStatus = QUIC_STREAM_SENDING_SEND;

    return SUCCESS;
}

int quicReceiveStreamRead(uint16_t connectionId, uint16_t streamId, uint8_t *cache, uint32_t len) {
    /* get connection item first */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connectionId);
    if (connItem == NULL) {
        DEBUG_PRINT("Error in quicReceiveStreamRead, connection item is not exist.");
        return ERROR;
    }
    /* get stream item */
    QUIC_Read_Stream_Item_t *streamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, streamId);
    if (streamItem == NULL) {
        DEBUG_PRINT("Error in quicReceiveStreamRead, stream item is not exist.");
        return ERROR;
    }
    /* read data from stream buffer */
    uint32_t restLen = len;
    uint32_t curPos = 0;
    int res = SUCCESS;
    // QUIC_Stream_Receive_Buffer_t *dataBuffer = &streamItem->dataBuffer;
    do {
        if (!streamItem->dataBuffer.isIntegrity) {
            return 1; // front streams' all data is read ready, but the rear stream's data is not complete, we still think data continuous streams in front are done
        }
        xSemaphoreTake(quicServerNode.mu, portMAX_DELAY);
        res = streamItem->readReceiveBuffer(&streamItem->dataBuffer, cache + curPos, restLen); // the res is total read len
        xSemaphoreGive(quicServerNode.mu);
        if (res == ERROR) {
            DEBUG_PRINT("quicReceiveStreamRead: read data from stream buffer failed.\n");
            return ERROR;
        }
        restLen -= res;
        curPos += res;
        streamItem->dataBuffer.consumedOffset += res;
        if (streamItem->dataBuffer.consumedOffset == streamItem->dataBuffer.receiveBlockList.tail->offset + streamItem->dataBuffer.receiveBlockList.tail->length) {
            streamItem->receivingStatus = QUIC_STREAM_RECEIVING_DATA_READY;
        }
        if(restLen == 0) break;
        // dataBuffer = (QUIC_Stream_Receive_Buffer_t *)dataBuffer->nextDataBuffer;
        streamItem = (QUIC_Read_Stream_Item_t *)streamItem->nextStreamItem;
    } while (streamItem != NULL);

    if (streamItem == NULL) return 1; // all data in this stream group is done, sliced but not even send one packet is no possible

    return 0; // data in stream is prepared ready, but user's cache is not enough, so we can't read all data in one time
}

/*
 * remove the stream's data buffer, and then check which streams on the connection can be released
 * @param connID: connection ID
 * @param streamID: stream ID
 * @return: SUCCESS or ERROR
 */
int quicReceiveStreamClose(uint16_t connectionId, uint16_t streamId) {
    /* get connection */
    /* get connection item first */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, connectionId);
    if (connItem == NULL) {
        DEBUG_PRINT("quicReceiveStreamClose: connection item is not exist.\n");
        return ERROR;
    }
    /* get stream item */
    QUIC_Read_Stream_Item_t *streamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, streamId); // it must be stream head
    if (streamItem == NULL) {
        DEBUG_PRINT("quicReceiveStreamClose: stream item is not exist.\n");
        return ERROR;
    }
    if (!streamItem->isHeadStream) {
        DEBUG_PRINT("quicReceiveStreamClose: stream item is not head stream.\n");
    }
    if (streamItem->receivingStatus != QUIC_STREAM_RECEIVING_DATA_READY) {
        DEBUG_PRINT("quicReceiveStreamClose: stream item is not ready to close.\n");
        return ERROR;
    }
    while (streamItem != NULL && streamItem->receivingStatus == QUIC_STREAM_RECEIVING_DATA_READY){
        /* free stream's data buffer */
        removeStreamReceiveBuffer(streamItem); // free all data buffer to free list
        /* get next group stream */
        QUIC_Read_Stream_Item_t *nextStreamItem = (QUIC_Read_Stream_Item_t *)streamItem->nextStreamItem;
        /* determines if it is the last stream in the group */
        if (nextStreamItem == NULL && streamItem->isHeadStream && streamItem->nextStreamID == 0) {
            /* if it is the last stream in the group, then remove it, free it's space */
            quicReadStreamItemRemove(&connItem->readStreams.streamsMap, streamItem->streamId);
            streamItem = NULL;
            break;
        }
        /* determine if it has a next uncreated stream that should exist at a later time */
        if (nextStreamItem == NULL && streamItem->isHeadStream && streamItem->nextStreamID != 0) {
            break;
        }
        /* hand over head to next group stream, if possible */
        if (nextStreamItem != NULL) {
            nextStreamItem->isHeadStream = true;
            nextStreamItem->prevStreamID = 0;
            nextStreamItem->prevStreamItem = NULL;
            nextStreamItem->streamGroupId = streamItem->streamGroupId;
            /* if be possible, remove current stream item, free it's space */
            quicReadStreamItemRemove(&connItem->readStreams.streamsMap, streamItem->streamId);
            streamItem = NULL;
        }
        /* next turn */
        streamItem = nextStreamItem;
    }

    return SUCCESS;
}

int quicClientResendData(QUIC_Packet_Info_Node_t *packetInfo) {
    /*
     * Steps:
     * 1. Prepare data packet
     * 2. Generate 1-RTT packet
     * 3. Generate stream frame
     * 4. Send packet
     */
    /* get connection item first */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packetInfo->connId);
    if (connItem == NULL) {
        DEBUG_PRINT("Error in quicClientResendData, connection item is not exist.");
        return ERROR;
    }
    /* prepare data packet */
    UWB_Data_Packet_t dataTxPacket;
    dataTxPacket.header.type = UWB_DATA_MESSAGE_QUIC;
    dataTxPacket.header.srcAddress = quicClientNode.me;
    dataTxPacket.header.destAddress = connItem->peer;
    dataTxPacket.header.ttl = 10;
    dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
    /* Generate packets */
    int packetPos = 0;
    /* Generate 1-RTT packet header */
    Quic_One_RTT_Packet_t *oneRTTPacket = (Quic_One_RTT_Packet_t *) &dataTxPacket.payload[packetPos];
    packetPos = quicGenerateOneRTTPacket(oneRTTPacket, connItem->dstConnId, QUIC_CLIENT);
    /* Generate frames */
    int framePos = 0;
    /* Generate stream frame */
    framePos += quicGenerateResendStreamFrame(oneRTTPacket, framePos, packetInfo);

    packetPos += framePos;
    dataTxPacket.header.length += packetPos;

    /* no need to handle ack(packet info) */
    /* Send packet */
    DEBUG_PRINT("quicClientResendData: resend data packet.\n");
    uwbSendDataPacketBlock(&dataTxPacket);
    /* return */
    return SUCCESS;
}

int quicReadStreamClear(QUIC_Read_Stream_Item_t *streamItem) {
    if (streamItem == NULL) {
        DEBUG_PRINT("quicReadStreamClear: stream item is not exist.\n");
        return ERROR;
    }
    /* free stream's data buffer */
    removeStreamReceiveBuffer(streamItem); // free all data buffer to free list
    /* free stream's space */
    free(streamItem);
    /* at this point, the node in the map is not deleted, need to be deleted outside */
    /* return */
    return SUCCESS;
}

int quicSendStreamClear(QUIC_Send_Stream_Item_t *streamItem) {
    if (streamItem == NULL) {
        DEBUG_PRINT("quicSendStreamClear: stream item is not exist.\n");
        return ERROR;
    }
    /* free stream's data buffer list */
    DataBlock_t *currBlock = streamItem->dataBuffer.pendingBlockList.head;
    while (currBlock != NULL) {
        DataBlock_t *nextBlock = currBlock->next;
        dataBlockClear(currBlock);
        currBlock = nextBlock;
    }
    currBlock = streamItem->dataBuffer.unackedBlockList.head;
    while (currBlock != NULL) {
        DataBlock_t *nextBlock = currBlock->next;
        dataBlockClear(currBlock);
        currBlock = nextBlock;
    }
    /* free stream's space */
    free(streamItem);
    /* at this point, the node in the map is not deleted, need to be deleted outside */
    /* return */
    return SUCCESS;

}

int quicSendStreamClose(uint16_t connectionId, uint16_t streamId) {
    /* get connection item first */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connectionId);
    if (connItem == NULL) {
        DEBUG_PRINT("quicSendStreamClose: connection item is not exist.\n");
        return ERROR;
    }
    /* get stream item */
    QUIC_Send_Stream_Item_t *streamItem = connItem->sendStreams.streamItemGet(&connItem->sendStreams.streamsMap, streamId);
    if (streamItem == NULL) {
        DEBUG_PRINT("quicSendStreamClose: stream item is not exist.\n");
        return ERROR;
    }
    QUIC_Stream_Send_Buffer_t *streamBuffer = &streamItem->dataBuffer;
    /* free stream's data buffer list, give back the blocks to the connection */
    DataBlock_t *currBlock = streamItem->dataBuffer.pendingBlockList.head;
    while (currBlock != NULL) {
        DataBlock_t *nextBlock = currBlock->next;
        /* remove the block to free list */
        currBlock->next = streamBuffer->freeBlocksHead->next;
        streamBuffer->freeBlocksHead->next = currBlock;

        currBlock = nextBlock;
    }
    currBlock = streamItem->dataBuffer.unackedBlockList.head;
    while (currBlock != NULL) {
        DataBlock_t *nextBlock = currBlock->next;
        /* remove the block to free list */
        currBlock->next = streamBuffer->freeBlocksHead->next;
        streamBuffer->freeBlocksHead->next = currBlock;

        currBlock = nextBlock;
    }
    /* remove stream item */
    quicSendStreamItemRemove(&connItem->sendStreams.streamsMap, streamId);
    /* return */
    return SUCCESS;
}

/* Transform Information Operations */
/*
 *  add stream info to the transport info, let client node know which stream is sending data
 *  @param connID: connection ID
 *  @param streamID: stream ID
 *  @return: SUCCESS or ERROR
 */
int quicTransformInfoAdd(uint16_t connID, uint16_t streamID) {
    int index;
    for(index = 0; index < QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT; index++) {
        if(quicClientNode.transportInfo[index].connectionID == 0) {
            quicClientNode.transportInfo[index].connectionID = connID;
            quicClientNode.transportInfo[index].streamID = streamID;
            break;
        }
    }
    if(index == QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT) {
        DEBUG_PRINT("quicTransformInfoAdd: stream number is over the largest stream number.\n");
        return ERROR;
    }
    return SUCCESS;
}

/*
 *  delete stream info from the transport info, let client node know which stream is done
 *  @param connID: connection ID
 *  @param streamID: stream ID
 *  @return: SUCCESS or ERROR
 */
int quicTransformInfoDelete(uint16_t connID, uint16_t streamID) {
    int index;
    for(index = 0; index < QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT; index++) {
        if(quicClientNode.transportInfo[index].connectionID == connID && quicClientNode.transportInfo[index].streamID == streamID) {
            quicClientNode.transportInfo[index].connectionID = 0;
            quicClientNode.transportInfo[index].streamID = 0;
            break;
        }
    }
    if(index == QUIC_INITIAL_MAX_STREAMS_UNI_DEFAULT) {
        DEBUG_PRINT("quicTransformInfoDelete: stream info is not exist.\n");
        return ERROR;
    }

    return SUCCESS;
}

/* TODO:
 * 2. 重传有两种，一种是超时重传，一种是接收到ACK后重传。
 * 4. 状态转移的判断还需要改进。
 */
// TODO: code review, 1-RTT Send and receive