/* USING RECOMMEND:
1. The bucket size is an integer multiple of 2;
2. It is best to fix the size of the map at once, 
because map space reallocation is very resource-intensive.*/

#ifndef __TOOLS_H__
#define __TOOLS_H__

/* For FreeRTOS, we config malloc and free as below. */
#define MAP_MALLOC pvPortMalloc
#define MAP_FREE vPortFree

typedef enum {
    MAP_TYPE_VOID_PTR,    // void *
    MAP_TYPE_CHAR_PTR,    // char *
    MAP_TYPE_INT,         // int
    MAP_TYPE_CHAR,        // char
    MAP_TYPE_FLOAT,       // float
    MAP_TYPE_DOUBLE,      // double
    MAP_TYPE_QUIC_CLIENT_CONN,   // QUIC_Client_Conn_Item_t
    MAP_TYPE_QUIC_SERVER_CONN,   // QUIC_Server_Conn_Item_t
} MAP_TYPE;

typedef enum {
    MAP_NOT_COPY_ADDR,
    MAP_COPY_ADDR
} MAP_COPY_ADDR_TYPE;

typedef struct {
    unsigned hash;
    void *value;
    struct Map_Node_t *next;
    /* char key[] and char value[] are exist, since mapCreateNode allocate space for them */
} Map_Node_t;

typedef struct {
    struct Map_Node_t **buckets;
    uint16_t bucketNumber;
    uint16_t nodeNumber;
} Map_Base_t;

typedef struct {
    unsigned bucketIndex;
    Map_Node_t *node;
} Map_Iter_t;

typedef struct {
    Map_Base_t mapBase;
    uint16_t typeSize;
    uint8_t isCpyAddr; /* when 0, pass values directly to a function, when 1, Copy the value from the address to the function. */
} Map_t;

/* Macro Function */
#define mapClear(map) \
    mapClear_(&(map)->base)
#define mapRemove(map, key) \
    mapRemove_(&(map)->base, key)
#define mapIter(map) \
    mapIter_()
#define mapNext(map, iter) \
    mapNext_(&(map)->base, iter)

/* Functions */
void mapInit(Map_t *instance, MAP_TYPE type, uint8_t isCpyAddr, uint16_t bucketNumber, int size);
void *mapGet(const Map_t *map, const char *key);
int mapSet(Map_t *map, const char *key, void *value, uint16_t valueSize);
void mapRemove_(Map_Base_t *map, const char *key);
Map_Iter_t mapIter_(void);
const char *mapNext_(Map_Base_t *map, Map_Iter_t *iter);
void mapClear_(Map_Base_t *map);

#endif
