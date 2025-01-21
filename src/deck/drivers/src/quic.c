#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "debug.h"
#include "system.h"
#include "quic.h"

#include <assert.h>

#include "routing.h"
#include "quicTools.h"

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
static TaskHandle_t quicRxTaskHandle;
static TaskHandle_t quicTxTaskHandle;
static QUIC_Node_t quicClientNode;
static QUIC_Node_t quicServerNode;
static uint16_t quicSrcConnId = 1;
static uint16_t quicTempSrcConnId = 0; /* There can only be one at the same time */
static SemaphoreHandle_t quicConnIdMutex;
static uint16_t quicStreamId = 1;
static SemaphoreHandle_t quicStreamIdMutex;

/* Local functions */
static uint16_t getNextSrcConnId() {
    xSemaphoreTake(quicConnIdMutex, M2T(0));
    const uint16_t nextId = quicSrcConnId++;
    xSemaphoreGive(quicConnIdMutex);
    return nextId;
}

static uint16_t getNextStreamId() {
    xSemaphoreTake(quicStreamIdMutex, M2T(0));
    const uint16_t nextId = quicStreamId++;
    xSemaphoreGive(quicStreamIdMutex);
    return nextId;
}

/* Global buffer operations */
static DataBlock_t *getStreamBufferDataBlock(void *streamBuffer_, const uint32_t minSize, const QUIC_NODE_STATUS status) {
    if(status == QUIC_CLIENT) {
        QUIC_Stream_Receive_Buffer_t *streamBuffer = streamBuffer_;
        if(streamBuffer->freeBlocks) {
            DataBlock_t *prev = NULL;
            DataBlock_t *curr = streamBuffer->freeBlocks;

            while(curr) {
                if(curr->capacity >= minSize) {
                    if(prev) prev->next = curr->next;
                    else streamBuffer->freeBlocks = curr->next;
                    curr->next = NULL;
                    return curr;
                }
                prev = curr;
                curr = curr->next;
            }
        }
    }
    else if (status == QUIC_SERVER) {
        QUIC_Stream_Send_Buffer_t *streamBuffer = streamBuffer_;
        if(streamBuffer->freeBlocks) {
            DataBlock_t *prev = NULL;
            DataBlock_t *curr = streamBuffer->freeBlocks;

            while(curr) {
                if(curr->capacity >= minSize) {
                    if(prev) prev->next = curr->next;
                    else streamBuffer->freeBlocks = curr->next;
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
static int writeStreamSendBuffer(QUIC_Stream_Send_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength) {
    if(streamBuffer->sendOffset + dataLength > streamBuffer->maxSendOffset) { // flow control
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
    block->offset = streamBuffer->sendOffset;
    /* add the block to the send list */
    if(streamBuffer->pendingBlockList.head == NULL) {
        streamBuffer->pendingBlockList.head = block;
    } else {
        streamBuffer->pendingBlockList.tail->next = block;
    }
    streamBuffer->pendingBlockList.tail = block;
    streamBuffer->pendingBlockList.count++;
    streamBuffer->sendOffset += dataLength;
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
    if (block->isFin) sendingData->isFin = true;
    else sendingData->isFin = false;
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
    streamBuffer->sendOffset += sendLength;
    return (int)sendLength;
}

static int ackStreamSendBuffer(QUIC_Stream_Send_Buffer_t *streamBuffer, const uint32_t offset, const uint32_t length) { // TODO: modify
    if(streamBuffer == NULL) {
        DEBUG_PRINT("ackStreamSendBuffer: data not complement.\n");
        return ERROR;
    }
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
            currBlock->next = streamBuffer->freeBlocks;
            streamBuffer->freeBlocks = currBlock;

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
    if(streamBuffer->readOffset >= streamBuffer->consumedOffset) {
        DEBUG_PRINT("writeStreamReceiveBuffer: no data can read\n");
        return ERROR;
    }
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
            /* remove form the reception block list */
            if(prevBlock != NULL) prevBlock->next = currBlock->next;
            else streamBuffer->receiveBlockList.head = currBlock->next;
            if(currBlock == streamBuffer->receiveBlockList.tail) streamBuffer->receiveBlockList.tail = prevBlock;
            streamBuffer->receiveBlockList.count--;
            /* move the released block to the free list */
            DataBlock_t *freeBlock = currBlock;
            currBlock = currBlock->next;
            freeBlock->next = streamBuffer->freeBlocks;
            streamBuffer->freeBlocks = freeBlock;
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
                nextBlock->next = streamBuffer->freeBlocks;
                streamBuffer->freeBlocks = nextBlock;
                nextBlock->length = 0;
            }
        }
        currBlock = currBlock->next;
    }
    return SUCCESS;
}

static int writeStreamReceiveBuffer(QUIC_Stream_Receive_Buffer_t *streamBuffer, const uint8_t *data, const uint32_t dataLength, const uint32_t offset) {
    if(offset + dataLength > streamBuffer->maxReceiveOffset) { // exceed max receive window
        DEBUG_PRINT("writeStreamReceiveBuffer: maxReceiveOffset exceeded\n");
        return ERROR;
    }
    DataBlock_t *block = getStreamBufferDataBlock(streamBuffer, dataLength, QUIC_SERVER);
    if(block == NULL) { // no more space
        DEBUG_PRINT("writeStreamReceiveBuffer: no block available\n");
        return ERROR;
    }
    memcpy(block->data, data, dataLength);
    block->length = dataLength;
    block->offset = offset;
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
    packetInfoNode->retransmitCount = 0;
    packetInfoNode->dataBlock = sendStreamItem->dataBuffer.unackedBlockList.tail;
    packetInfoNode->offset = packetInfoNode->dataBlock->offset;
    packetInfoNode->length = packetInfoNode->dataBlock->length;

    return SUCCESS;
}

static int quicPacketInfoNodeInsert(QUIC_Packet_Info_Manager_t *packetInfoManager, QUIC_Packet_Info_Node_t *packetInfoNode) {
    if(packetInfoManager == NULL || packetInfoNode == NULL) {
        DEBUG_PRINT("quicPacketInfoNodeInsert: data not complement.\n");
        return ERROR;
    }
    if(packetInfoNode->packetNumber <= packetInfoManager->minimumPacketUnackedNumber) {
        DEBUG_PRINT("quicPacketInfoNodeInsert: packet number is less than minimum packet acked number.\n");
        return SUCCESS;
    }
    const int res = insertRBTree(packetInfoManager->packetInfoRBTree, (int)packetInfoNode->packetNumber, packetInfoNode, sizeof(QUIC_Packet_Info_Node_t));
    if(res == ERROR) {
        DEBUG_PRINT("quicPacketInfoNodeInsert: insert failed.\n");
        return ERROR;
    }
    if(packetInfoNode->packetNumber > packetInfoManager->largestPacketNumber) packetInfoManager->largestPacketNumber = packetInfoNode->packetNumber;
    packetInfoManager->packetInfoNodeCount++;

    return SUCCESS;
}

static int quicPacketInfoNodeDelete(QUIC_Packet_Info_Manager_t *packetInfoManager, const uint32_t packetNumber, QUIC_Client_Conn_Item_t *clientConnItem) {
    if (packetInfoManager == NULL) {
        DEBUG_PRINT("quicPacketInfoNodeDelete: packetInfoManager is null.\n");
        return ERROR;
    }
    /* get packet info node and delete relative buffer's block */
    const RBNode_t *rbNode = searchRBTree(packetInfoManager->packetInfoRBTree, (int)packetNumber);
    if(rbNode == NULL) {
        DEBUG_PRINT("quicPacketInfoNodeDelete: packet info node not found.\n");
        return ERROR;
    }
    QUIC_Packet_Info_Node_t *packetInfoNode = rbNode->data;
    /* get stream item */
    QUIC_Send_Stream_Item_t *sendStreamItem = clientConnItem->sendStreams.streamItemGet(&clientConnItem->sendStreams.streamsMap, packetInfoNode->streamId);
    /* remove the block from the unacked block list */
    ackStreamSendBuffer(&sendStreamItem->dataBuffer, packetInfoNode->offset, packetInfoNode->length);

    /* delete packet info node */
    int res = deleteRBTree(packetInfoManager->packetInfoRBTree, (int)packetNumber);
    if(res == -1) {
        DEBUG_PRINT("quicPacketInfoNodeDelete: delete failed.\n");
        return ERROR;
    }
    if(packetNumber == packetInfoManager->minimumPacketUnackedNumber) {
        packetInfoNode = NULL; // reuse
        res = RBTreeMinimum(packetInfoManager->packetInfoRBTree, (void **)&packetInfoNode);
        if (res == -1) {
            DEBUG_PRINT("quicPacketInfoNodeDelete: get min packet number failed.\n");
            return ERROR;
        }
        if (res == 1) {
            DEBUG_PRINT("quicPacketInfoNodeDelete: no packet in the tree.\n"); // this means all packet acked
            packetInfoManager->minimumPacketUnackedNumber = 0; // all packet acked
            return SUCCESS;
        }
        if (packetInfoNode == NULL) {
            DEBUG_PRINT("quicPacketInfoNodeDelete: packetInfoNodePtr is null.\n");
            return ERROR;
        }
        packetInfoManager->minimumPacketUnackedNumber = packetInfoNode->packetNumber;
    }

    return SUCCESS;
}

static int quicPacketInfoManagerInit(QUIC_Packet_Info_Manager_t *packetInfoManager) {
    if (packetInfoManager == NULL) {
        DEBUG_PRINT("quicPacketInfoManagerInit: packetInfoManager is null.\n");
        return ERROR;
    }
    packetInfoManager->largestPacketNumber = 0; // no packet
    packetInfoManager->minimumPacketUnackedNumber = 0; // all packet acked
    packetInfoManager->packetInfoNodeCount = 0;
    packetInfoManager->packetInfoRBTree = createRBTree();

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

// TODO: add ack ranges functions

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

static void quicConnItemSet(Map_t *connItemsMap, const uint16_t connId, void *connItem, const QUIC_NODE_STATUS status) {
    char mapKey[10] = {0};
    itoa(connId, mapKey, 10);
    if (status == QUIC_CLIENT) {
        mapSet(connItemsMap, mapKey, connItem, sizeof(QUIC_Client_Conn_Item_t));
    }
    if (status == QUIC_SERVER) {
        mapSet(connItemsMap, mapKey, connItem, sizeof(QUIC_Server_Conn_Item_t));
    }
}

static void *quicConnItemGet(const Map_t *connItemsMap, const uint16_t connId) {
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

static void quicTxTask() {
    systemWaitStart();

    while(true)
    {
#ifdef QUIC_TEST_CLIENT
        quicClientSendConnRequest(0);
#endif
        vTaskDelay(M2T(3000));
    }
}

static void quicRxTask() {
    systemWaitStart();

    UWB_Data_Packet_t dataRxPacket;
    while(true) {
        if(uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_QUIC, &dataRxPacket)) {
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
                            packetOffset = quicProcessInitialPacket(packet, peer);
                            break;
                        case QUIC_ZERO_RTT_PACKET:
                            break;
                        case QUIC_HANDSHAKE_PACKET:
                            DEBUG_PRINT("In quicRxTask: Handshake packet received.\n");
                            packetOffset = quicProcessHandshakePacket(packet, peer);
                            break;
                        default:
                            DEBUG_PRINT("Error in quicRxTask, long packet type is not exist.\n");
                            break;
                    }
                }
                if (headerForm == QUIC_SHORT_HEADER){
                    // Quic_One_RTT_Packet_t *packet = (Quic_One_RTT_Packet_t *) &dataRxPacket.payload[curPosLen];
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
    quicConnItemsMapInit(&quicClientNode.conns, QUIC_CLIENT);
    // If I receive a packet from client, or send a packet to client, then I am server
    quicServerNode.me = uwbGetAddress();
    quicServerNode.mu = xSemaphoreCreateMutex();
    quicServerNode.conns.size = 0;
    quicServerNode.conns.capacity = QUIC_CONNECTION_NUMBER_MAX;
    quicConnItemsMapInit(&quicServerNode.conns, QUIC_SERVER);

    xTaskCreate(quicRxTask, ADHOC_DECK_QUIC_RX_TASK_NAME, UWB_TASK_STACK_SIZE, NULL, ADHOC_DECK_TASK_PRI, &quicRxTaskHandle);
    /* Test */
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
    /* Create new connection for client */
    QUIC_Server_Conn_Item_t connItem = {0};
    connItem.peer = peer;
    connItem.connId = getNextSrcConnId();
    connItem.currentState = QUIC_SERVER_CONN_STATE_INITIAL;
    connItem.dstConnId = packet->header.srcConnId;
    quicServerNode.conns.connItemSet(&quicServerNode.conns.connItemsMap, connItem.connId, &connItem, QUIC_SERVER);
    quicServerNode.conns.size++;

    quicTempSrcConnId = connItem.connId; /* Will be used when server reply */
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
        frame->header.ACKDelay = packet->header.headerForm == QUIC_LONG_HEADER ? 0 : QUIC_ACK_DELAY_MAX; // TODO: set timer and calculate delay;
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
        frame->header.ACKDelay = packet->header.headerForm == QUIC_LONG_HEADER ? 0 : QUIC_ACK_DELAY_MAX; // TODO: set timer and calculate delay;
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

/* Generate stream frame, buffer data is filled to the packet according to the remaining space of the packet */
int quicGenerateStreamFrame(Quic_One_RTT_Packet_t *packet, const uint16_t framePos, const uint16_t connID, const uint16_t streamID, const uint32_t restLength) {
    if(packet->header.status != QUIC_CLIENT) {
        DEBUG_PRINT("Error in quicGenerateStreamFrame, peer status is not client.\n");
    }
    if(packet->header.length + sizeof(Quic_Stream_Frame_Header_t) > QUIC_LONG_PACKET_PAYLOAD_SIZE_MAX) {
        DEBUG_PRINT("Error in quicGenerateStreamFrame, payload overflow.\n");
    }
    /* rest length */
    const uint32_t payloadLength = restLength - sizeof(Quic_Stream_Frame_Header_t);
    /* Get stream buffer */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, connID);
    QUIC_Send_Stream_Item_t *streamItem = connItem->sendStreams.streamItemGet(&connItem->sendStreams.streamsMap, streamID);
    QUIC_Stream_Sending_Data_t sendingData = {0};
    streamItem->readSendBuffer(&streamItem->dataBuffer, &sendingData, payloadLength, packet->header.packetNumber);
    /* Generate frame header */
    Quic_Stream_Frame_t *frame = (Quic_Stream_Frame_t *) &packet->packetPayload[framePos];
    memset(frame, 0, sizeof(Quic_Stream_Frame_Header_t));
    frame->header.type = sendingData.isFin ? QUIC_FRAME_STREAM_FIN : QUIC_FRAME_STREAM; /* stream or stream fin */
    frame->header.streamID = getNextStreamId();
    frame->header.offset = sendingData.offset;
    frame->header.length = sizeof(Quic_Stream_Frame_Header_t) + payloadLength;
    /* Write payload */
    memcpy(&frame->data[0], sendingData.data, payloadLength);

    return (int)sizeof(Quic_Stream_Frame_Header_t) + (int)payloadLength;
}

int quicHandleStreamFrame(const Quic_One_RTT_Packet_t *packet, const int pos){
    if(packet->header.status != QUIC_SERVER) {
        DEBUG_PRINT("Error in quicHandleStreamFrame, peer status is not server.\n");
    }
    const Quic_Stream_Frame_t *frame = (Quic_Stream_Frame_t *) &packet->packetPayload[pos];
    /* Get stream from connection item */
    QUIC_Server_Conn_Item_t *connItem = quicServerNode.conns.connItemGet(&quicServerNode.conns.connItemsMap, packet->header.dstConnId);
    QUIC_Read_Stream_Item_t *streamItem = connItem->readStreams.streamItemGet(&connItem->readStreams.streamsMap, frame->header.streamID);
    if(streamItem == NULL) {
        QUIC_Read_Stream_Item_t streamItem_ = {0};
        streamItem_.streamId = frame->header.streamID;
        streamItem_.dataBuffer.receiveBlockList.head = NULL;
        streamItem_.dataBuffer.receiveBlockList.tail = NULL;
        streamItem_.dataBuffer.receiveBlockList.count = 0;
        streamItem_.dataBuffer.readOffset = 0;
        streamItem_.dataBuffer.consumedOffset = 0;
        streamItem_.dataBuffer.maxReceiveOffset = QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT;
        streamItem_.dataBuffer.freeBlocks = NULL;
        streamItem_.dataBuffer.receivedFin = false;
        streamItem = connItem->readStreams.streamItemSet(&connItem->readStreams.streamsMap, &streamItem_, frame->header.streamID);
        if(streamItem == NULL) {
            DEBUG_PRINT("Error in quicHandleStreamFrame, create new stream item failed, space is not enough.\n");
            return ERROR;
        }
        assert(streamItem != NULL);
    }
    streamItem->writeReceiveBuffer(&streamItem->dataBuffer, &frame->data[0], frame->header.length - sizeof(Quic_Stream_Frame_Header_t), frame->header.offset);
    // TODO: stream fin handle may modify
    if (frame->header.type == QUIC_FRAME_STREAM_FIN) streamItem->dataBuffer.receivedFin = true;

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
    frame->header.ACKDelay = QUIC_ACK_DELAY_MAX;
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
        }
    }
    packet->header.length += sizeof(Quic_ACK_Frame_Header_t) + frame->header.ACKRangeCount * sizeof(Quic_ACK_Range_t);
    if (packet->header.length - sizeof(Quic_Short_Packet_Header_t) >= QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX) {
        DEBUG_PRINT("quicGenerateOneRTTPacketACKFrame: payload overflow.");
        return ERROR;
    }
    return (int) sizeof(Quic_ACK_Frame_Header_t) + frame->header.ACKRangeCount * (int) sizeof(Quic_ACK_Range_t);
}

/* Only client will use this function,  */
int quicHandleOneRTTPacketACKFrame(const Quic_One_RTT_Packet_t *packet, const int pos) {
    /* get ack frame */
    const Quic_ACK_Frame_t *frame = (Quic_ACK_Frame_t *) &packet->packetPayload[pos];
    const int minAckedPacketNumber = frame->header.minimumACK; // minimum acked packet number
    /* get client connection item */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, packet->header.dstConnId);
    /* make all packet that number less than minAckedPacketNumber be acked */
    while (connItem->packetInfoManager.minimumPacketUnackedNumber <= minAckedPacketNumber) {
        connItem->packetInfoManager.packetInfoNodeDelete((struct QUIC_Packet_Info_Manager_t*)&connItem->packetInfoManager, connItem->packetInfoManager.minimumPacketUnackedNumber, (struct QUIC_Client_Conn_Item_t*)connItem);
        connItem->packetInfoManager.minimumPacketUnackedNumber++;
    }
    /* ack packets that server has received */
    for (int i = 0; i < frame->header.ACKRangeCount; i++) {
        int gap = frame->ACKRange[i].gap;
        int len = frame->ACKRange[i].ackRangeLength;
        // TODO: now
    }

    return SUCCESS;
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

int quicProcessInitialPacket(const Quic_Long_Packet_t *initialPacket, const UWB_Address_t peer) {
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

int quicProcessHandshakePacket(Quic_Long_Packet_t *handshakePacket, const UWB_Address_t peer) {
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

    /* The ACK(Packet info) handle will be set after generate stream frame */

    packet->header.length = sizeof(Quic_Short_Packet_Header_t);

    return SUCCESS;
}

int quicProcessOneRTTPacket(const Quic_One_RTT_Packet_t *oneRTTPacket) {
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
        case QUIC_FRAME_ACK:
            DEBUG_PRINT("quicProcessOneRTTPacket: handle ack frame.\n");
            // Only client need it
            // TODO: add 1-RTT packet ack frame handle
            frameOffset = quicHandleOneRTTPacketACKFrame(oneRTTPacket, curPosLen);
            break;
        default:
            DEBUG_PRINT("quicProcessOneRTTPacket: Error in process handshake packet, frame type is not exist.\n");
            return ERROR;
        }
        if (frameOffset == ERROR) return ERROR;
        curPosLen += frameOffset;
    }
    /* Receive ack handle, only server need it */
    if (oneRTTPacket->header.status == QUIC_CLIENT) {
        QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, oneRTTPacket->header.dstConnId);
        if (connItem == NULL) {
            DEBUG_PRINT("quicProcessOneRTTPacket: Error in process handshake packet, connection item is not exist.\n");
            return ERROR;
        }
        if (oneRTTPacket->header.packetNumber > connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK) {
            quicACKRangesAddGap(connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges, oneRTTPacket->header.packetNumber - connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK - 1);
            connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK = oneRTTPacket->header.packetNumber;
        }
        else if (oneRTTPacket->header.packetNumber == connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].largestACK) {
            DEBUG_PRINT("quicProcessOneRTTPacket: packet number is equal to the largestACK, no need to handle.\n");
        }
        else { // packet number < largestACK
            const int res = quicACKRangesUpdateLen(connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].ackRanges, oneRTTPacket->header.packetNumber - connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].minimumACK - 1);
            if (res == ERROR) {
                DEBUG_PRINT("quicProcessOneRTTPacket: Error in update ACK ranges.\n");
                return ERROR;
            }
            connItem->packetReceiveWindow[QUIC_ONE_RTT_PACKET_ACK].minimumACK += res;
        }
    }

    return SUCCESS;
}

/* Message Operations */
int quicClientSendConnRequest(const UWB_Address_t peer) {
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
    /* since memset, packetTuples are already set 0. */
    quicClientNode.conns.connItemSet(&quicClientNode.conns.connItemsMap, srcConnId, &connItem_, QUIC_CLIENT);
    quicClientNode.conns.size++;
    /* Find connection with peer */
    QUIC_Client_Conn_Item_t *connItem = quicClientNode.conns.connItemGet(&quicClientNode.conns.connItemsMap, srcConnId);
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
     * 5. Generate 1-RTT packet
     * 6. Generate stream frame
     * 7. Send packet
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
    /* Generate 1-RTT packet */
    // TODO: Add 1-RTT packet

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
     * 4. Generate 1-RTT packet
     * 5. Generate stream frame
     * 6. Send packet
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
    /* Generate 1-RTT packet */
    // TODO: Add 1-RTT packet

    dataTxPacket.header.length += packetPos;

    DEBUG_PRINT("In quicServerSendConnDone: handshake and 1-RTT packets send.\n"); /* add parameters according to the debugging situation. */
    uwbSendDataPacketBlock(&dataTxPacket);
    quicStateTransport(connId, QUIC_SERVER);

    return SUCCESS;
}

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
    /* check stream ID is exist or not, if exists, get the stream item, if not, then create a new stream */
    const QUIC_Send_Stream_Item_t *streamItem = connItem->sendStreams.streamItemGet(&connItem->sendStreams.streamsMap, streamID);
    // TODO: modify code, in upper layer, we need to check whether the stream is exist or not
    if (streamItem == NULL) { // before send data, stream item and data have been created anyway
        QUIC_Send_Stream_Item_t streamItem_ = {0};
        streamItem_.streamId = streamID;
        streamItem_.dataBuffer.pendingBlockList.head = NULL;
        streamItem_.dataBuffer.pendingBlockList.tail = NULL;
        streamItem_.dataBuffer.pendingBlockList.count = 0;
        streamItem_.dataBuffer.unackedBlockList.head = NULL;
        streamItem_.dataBuffer.unackedBlockList.tail = NULL;
        streamItem_.dataBuffer.unackedBlockList.count = 0;
        streamItem_.dataBuffer.freeBlocks = NULL;
        streamItem_.dataBuffer.maxSendOffset = QUIC_INITIAL_MAX_STREAM_DATA_UNI_DEFAULT;
        streamItem_.dataBuffer.sendOffset = 0;
        streamItem_.dataBuffer.sentFin = false;
        streamItem = connItem->sendStreams.streamItemSet(&connItem->sendStreams.streamsMap, &streamItem_, streamID);
        if (streamItem == NULL) {
            DEBUG_PRINT("Error in quicClientSendData, create new stream item failed, space is not enough.");
            return ERROR;
        }
        assert(streamItem != NULL);
    }
    /* write data to stream frame */
    const uint32_t restLength = QUIC_ONE_RTT_PACKET_PAYLOAD_SIZE_MAX - sizeof(Quic_Short_Packet_Header_t);
    framePos += quicGenerateStreamFrame(oneRTTPacket, framePos, connItem->connId, streamID, restLength);

    packetPos += framePos;
    dataTxPacket.header.length += packetPos;
    /* handle ack(packet info) */
    QUIC_Packet_Info_Node_t packetInfoNode = {0};
    quicPacketInfoNodeInit(&packetInfoNode, oneRTTPacket, streamItem);
    QUIC_Packet_Info_Manager_t *packetInfoManager = &connItem->packetInfoManager;
    packetInfoManager->packetInfoNodeInsert((struct QUIC_Packet_Info_Manager_t*)&packetInfoManager, &packetInfoNode);
    /* Send packet */
    DEBUG_PRINT("quicClientSendData: send data packet.\n");
    uwbSendDataPacketBlock(&dataTxPacket);
    /* return */
    return SUCCESS;
}

/* Quic Interaction Operations */
/* TODO:
 * 1. 当报文超过负载了怎么办？
 * 2. 重传有两种，一种是超时重传，一种是接收到ACK后重传。
 * 3. 考虑发送失败的纠错机制。
 * 4. 状态转移的判断还需要改进。
 */

// TODO: latest, coding 1-RTT packet, now coding receive part
// TODO: latest, timer, that send ack frame
// TODO: latest, in quicHandleOneRTTPacketACKFrame, handle ack frame, and update packet info