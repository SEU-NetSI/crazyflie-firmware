#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "FreeRTOS.h"
#include "tools.h"
#include "quic.h"

static unsigned mapHash(const char *str) {
    unsigned hash = 5381;
    while(*str) hash = ((hash << 5) + hash) ^ *str++;
    return hash;
}

static Map_Node_t *mapCreateNode(const char *key, void *value, int valueSize) {
    Map_Node_t *node;
    int keySize = strlen(key) + 1;
    int valueOffset = keySize + ((sizeof(void *) - keySize) % sizeof(void *)); /* Align according to the number of bits in the system */
    node = MAP_MALLOC(sizeof(*node) + valueOffset + valueSize);
    if(!node) return NULL;
    memcpy(node + 1, key, keySize);
    node->hash = mapHash(key);
    node->value = ((char *) (node + 1)) + valueOffset;
    memcpy(node->value, value, valueSize);
    return node;
}

static int mapGetBucketIndex(Map_Base_t *map, unsigned hash) {
    return hash & (map->bucketNumber - 1);
}

static void mapAddNode(Map_Base_t *map, Map_Node_t *node) {
    int index = mapGetBucketIndex(map, node->hash);
    node->next = map->buckets[index];
    map->buckets[index] = node;
}

static int mapResize(Map_Base_t *map, int bucketNumber) {
    Map_Node_t *head, *node, *next;
    Map_Node_t **buckets;
    int index;
    /* Chain all nodes together */
    head = NULL;
    index = map->bucketNumber;
    while(index--) {
        node = (map->buckets)[index];
        while(node) {
            next = node->next;
            node->next = head;
            head = node;
            node = next;
        }
    }
    /* Reset buckets */
    if(map->buckets != NULL) {
        MAP_FREE(map->buckets);
    }
    buckets = MAP_MALLOC(sizeof(&map->buckets) * bucketNumber);
    if(buckets != NULL) {
        map->buckets = buckets;
        map->bucketNumber = bucketNumber;
    }
    if(map->buckets) {
        memset(map->buckets, 0, sizeof(*map->buckets) * map->bucketNumber);
        /* Re-add nodes to buckets */
        node = head;
        while(node) {
            next = node->next;
            mapAddNode(map, node);
            node = next;
        }
    }
    return (buckets == NULL) ? -1 : 0;
}

static Map_Node_t **mapGetNodeRef(Map_Base_t *map, const char *key) {
    unsigned hash = mapHash(key);
    Map_Node_t **next;
    if(map->bucketNumber > 0) {
        next = &map->buckets[mapGetBucketIndex(map, hash)];
        while(*next) {
            if((*next)->hash == hash && !strcmp((char *) (*next + 1), key)) return next;
            next = &(*next)->next;
        }
    }
    return NULL;
}

void mapInit(Map_t *instance, MAP_TYPE type, uint8_t isCpyAddr, uint16_t bucketNumber) {
    memset(instance, 0, sizeof(Map_t));
    switch(type) {
        case MAP_TYPE_VOID_PTR        :{instance->typeSize = sizeof(void *);break;}
        case MAP_TYPE_CHAR_PTR        :{instance->typeSize = sizeof(char *);break;}
        case MAP_TYPE_INT             :{instance->typeSize = sizeof(int);break;}
        case MAP_TYPE_CHAR            :{instance->typeSize = sizeof(char);break;}
        case MAP_TYPE_FLOAT           :{instance->typeSize = sizeof(float);break;}
        case MAP_TYPE_DOUBLE          :{instance->typeSize = sizeof(double);break;}
        case MAP_TYPE_QUIC_CLIENT_CONN:{instance->typeSize = sizeof(QUIC_Client_Conn_Item_t);break;}
        case MAP_TYPE_QUIC_SERVER_CONN:{instance->typeSize = sizeof(QUIC_Server_Conn_Item_t);break;}
        default:break;
    }
    instance->isCpyAddr = isCpyAddr;
    assert((bucketNumber % 2) == 0);
    mapResize(&instance->mapBase, bucketNumber);
}

void *mapGet(Map_t *map, const char *key) {
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