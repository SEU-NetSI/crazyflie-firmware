#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "FreeRTOS.h"
#include "quicTools.h"

#include <debug.h>
#include <quic.h>

static unsigned mapHash(const char *str) {
    unsigned hash = 5381;
    while(*str) hash = ((hash << 5) + hash) ^ *str++;
    return hash;
}

static Map_Node_t *mapCreateNode(const char *key, const void *value, const int valueSize) {
    const unsigned int keySize = strlen(key) + 1;
    const unsigned int valueOffset = keySize + (sizeof(void *) - keySize) % sizeof(void *); /* Align according to the number of bits in the system */
    Map_Node_t *node = MAP_MALLOC(sizeof(*node) + valueOffset + valueSize);
    if(!node) return NULL;
    memcpy(node + 1, key, keySize);
    node->hash = mapHash(key);
    node->value = (char *) (node + 1) + valueOffset;
    memcpy(node->value, value, valueSize);
    return node;
}

static unsigned int mapGetBucketIndex(const Map_Base_t *map, const unsigned int hash) {
    return hash & (map->bucketNumber - 1);
}

static void mapAddNode(const Map_Base_t *map, Map_Node_t *node) {
    const unsigned int index = mapGetBucketIndex(map, node->hash);
    node->next = map->buckets[index];
    map->buckets[index] = (struct Map_Node_t *)node;
}

static int mapResize(Map_Base_t *map, const int bucketNumber) {
    Map_Node_t *head = NULL, *node, *next;
    int index = map->bucketNumber;;
    /* Chain all nodes together */
    while(index--) {
        if (map->buckets == NULL) break;
        node = (Map_Node_t *)map->buckets[index];
        while(node) {
            next = (Map_Node_t *)node->next;
            node->next = (struct Map_Node_t *)head;
            head = node;
            node = next;
        }
    }
    /* Reset buckets */
    if(map->buckets != NULL) {
        MAP_FREE(map->buckets);
    }
    Map_Node_t **buckets = MAP_MALLOC(sizeof(&map->buckets) * bucketNumber);
    if(buckets != NULL) {
        map->buckets = (struct Map_Node_t **)buckets;
        map->bucketNumber = bucketNumber;
    }
    if(map->buckets) {
        memset(map->buckets, 0, sizeof(*map->buckets) * map->bucketNumber);
        /* Re-add nodes to buckets */
        node = head;
        while(node) {
            next = (Map_Node_t *)node->next;
            mapAddNode(map, node);
            node = next;
        }
    }
    return (buckets == NULL) ? -1 : 0;
}

static Map_Node_t **mapGetNodeRef(const Map_Base_t *map, const char *key) {
    const unsigned int hash = mapHash(key);
    if(map->bucketNumber > 0) {
        Map_Node_t **next = (Map_Node_t **)&map->buckets[mapGetBucketIndex(map, hash)];
        while(*next) {
            if((*next)->hash == hash && !strcmp((char *) (*next + 1), key)) return next;
            next = (Map_Node_t **)&(*next)->next;
        }
    }
    return NULL;
}

void mapInit(Map_t *instance, MAP_TYPE type, uint8_t isCpyAddr, uint16_t bucketNumber, int size) {
    memset(instance, 0, sizeof(Map_t));
    switch(type) {
        case MAP_TYPE_VOID_PTR        :{instance->typeSize = sizeof(void *);break;}
        case MAP_TYPE_CHAR_PTR        :{instance->typeSize = sizeof(char *);break;}
        case MAP_TYPE_INT             :{instance->typeSize = sizeof(int);break;}
        case MAP_TYPE_CHAR            :{instance->typeSize = sizeof(char);break;}
        case MAP_TYPE_FLOAT           :{instance->typeSize = sizeof(float);break;}
        case MAP_TYPE_DOUBLE          :{instance->typeSize = sizeof(double);break;}
        case MAP_TYPE_QUIC_CLIENT_CONN:
        case MAP_TYPE_QUIC_SERVER_CONN:{instance->typeSize = size;break;}
        default:break;
    }
    instance->isCpyAddr = isCpyAddr;
    assert((bucketNumber % 2) == 0);
    mapResize(&instance->mapBase, bucketNumber);
}

void *mapGet(const Map_t *map, const char *key) {
    Map_Node_t **next = mapGetNodeRef(&map->mapBase, key);
    return next ? (*next)->value : NULL;
}

int mapSet(Map_t *map, const char *key, void *value, uint16_t valueSize) {
    int number, err;
    Map_Node_t **next, *node;
    if(valueSize == 0) valueSize = map->typeSize; /* In addition to copying the value in the address, you need to pass the data length, and other times you can pass 0 */
    /* If the exact key already exists, replace the value */
    next = mapGetNodeRef(&map->mapBase, key);
    if(next) {
        if(map->isCpyAddr) memcpy((*next)->value, value, valueSize);
        else memcpy((*next)->value, &value, map->typeSize);
        return 0;
    }
    /* Add new node */
    if(map->isCpyAddr) node = mapCreateNode(key, value, valueSize);
    else node = mapCreateNode(key, &value, map->typeSize);

    bool fail = false;
    if(node == NULL) fail = true;
    if(map->mapBase.nodeNumber >= map->mapBase.bucketNumber) {
        number = (map->mapBase.nodeNumber > 0) ? (map->mapBase.bucketNumber << 1) : 1;
        err = mapResize(&map->mapBase, number);
        if(err) fail = true;
    }
    if(fail) {
        if(node) MAP_FREE(node);
        return -1;
    }

    mapAddNode(&map->mapBase, node);
    map->mapBase.nodeNumber++;

    return 0;
}

void mapRemove_(Map_Base_t *map, const char *key) {
    Map_Node_t *node;
    Map_Node_t **next = mapGetNodeRef(map, key);
    if(next) {
        node = *next;
        *next = (*next)->next;
        MAP_FREE(node);
        map->nodeNumber--;
    }
}

Map_Iter_t mapIter_(void) {
    Map_Iter_t iter;
    iter.bucketIndex = -1;
    iter.node = NULL;
    return iter;
}

const char *mapNext_(Map_Base_t *map, Map_Iter_t *iter) {
    if(iter->node) {
        iter->node = iter->node->next;
        if(iter->node == NULL) goto nextBucket;
    } else {
        nextBucket:
        do {
            if(++iter->bucketIndex >= map->bucketNumber) return NULL;
            iter->node = map->buckets[iter->bucketIndex];
        } while(iter->node == NULL);
    }
    return (char *) (iter->node + 1);
} // TODO: debug

void mapClear_(Map_Base_t *map) {
    Map_Node_t *next, *node;
    int index;
    index = map->bucketNumber;
    while(index--) {
        node = map->buckets[index];
        while(node) {
            next = node->next;
            MAP_FREE(node);
            node = next;
        }
    }
    MAP_FREE(map->buckets);
}