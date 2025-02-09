/* USING RECOMMEND:
1. The bucket size is an integer multiple of 2;
2. It is best to fix the size of the map at once, 
because map space reallocation is very resource-intensive.*/

#ifndef __TOOLS_H__
#define __TOOLS_H__

/* For FreeRTOS, we config malloc and free as below. */
#define TOOL_MALLOC pvPortMalloc
#define TOOL_FREE vPortFree

/* MAP */
typedef enum {
    MAP_TYPE_VOID_PTR,    // void *
    MAP_TYPE_CHAR_PTR,    // char *
    MAP_TYPE_INT,         // int
    MAP_TYPE_CHAR,        // char
    MAP_TYPE_FLOAT,       // float
    MAP_TYPE_DOUBLE,      // double
    MAP_TYPE_QUIC_CLIENT_CONN,   // QUIC_Client_Conn_Item_t
    MAP_TYPE_QUIC_SERVER_CONN,   // QUIC_Server_Conn_Item_t
    MAP_TYPE_QUIC_SEND_STREAM,   // QUIC_Send_Stream_Item_t
    MAP_TYPE_QUIC_READ_STREAM,   // QUIC_Read_Stream_Item_t
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
    mapClear_(&(map)->mapBase)
#define mapRemove(map, key) \
    mapRemove_(&(map)->mapBase, key)
#define mapIter(map) \
    mapIter_()
#define mapNext(map, iter) \
    mapNext_(&(map)->mapBase, iter)

/* Functions */
void mapInit(Map_t *instance, MAP_TYPE type, uint8_t isCpyAddr, uint16_t bucketNumber, int size);
void *mapGet(const Map_t *map, const char *key);
void *mapSet(Map_t *map, const char *key, void *value, uint16_t valueSize);
void mapRemove_(Map_Base_t *map, const char *key);
Map_Iter_t mapIter_(void);
const char *mapNext_(Map_Base_t *map, Map_Iter_t *iter);
void mapClear_(Map_Base_t *map);

/* MEMORY BLOCK */
typedef struct DataBlock {
    uint8_t *data;
    uint32_t offset;
    uint32_t length; // the data length in the block
    uint32_t capacity; // max capacity of data length of the block
    struct DataBlock *next;
    bool isFin;
} DataBlock_t;

typedef struct BlockList {
    DataBlock_t *head;
    DataBlock_t *tail;
    uint32_t count;
} BlockList_t;

/* Functions */
DataBlock_t *dataBlockInit(uint32_t capacity);
void dataBlockClear(DataBlock_t *block);

/* RBTree */
#define RED   0
#define BLACK 1
/* Key Type */
typedef int   KeyType_t;
typedef int   DataSizeType_t;
typedef void* DataPtr_t;
/* RBTree Node */
typedef struct RBTreeNode {
    unsigned char color;
    KeyType_t key;
    DataSizeType_t size;
    DataPtr_t data;
    struct RBTreeNode *left;
    struct RBTreeNode *right;
    struct RBTreeNode *parent;
} RBNode_t;
/* Root of RBTree */
typedef struct {
    RBNode_t *node;
    uint16_t size;
    void *externResourcePtr;
} RBRoot_t;
/* Functions */
RBRoot_t* createRBTree(void); // create a new RBTree
void clearRBTree(RBRoot_t *root); // destroy a RBTree
int insertRBTree(RBRoot_t *root, KeyType_t key, DataPtr_t data, DataSizeType_t dataSize); // insert a node to RBTree
int deleteRBTree(RBRoot_t *root, KeyType_t key); // delete a node from RBTree with key
RBNode_t* searchRBTree(const RBRoot_t *root, KeyType_t key); // search a node in RBTree
int RBTreeMinimum(const RBRoot_t *root, DataPtr_t *dataPtrPtr); // find the minimum node in RBTree
int RBTreeMaximum(const RBRoot_t *root, DataPtr_t *dataPtrPtr); // find the maximum node in RBTree

#endif
