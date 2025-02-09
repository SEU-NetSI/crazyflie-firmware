#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "FreeRTOS.h"
#include "quicTools.h"

#include <debug.h>
#include <quic.h>

/* MAP */
static unsigned mapHash(const char *str) {
    unsigned hash = 5381;
    while(*str) hash = ((hash << 5) + hash) ^ *str++;
    return hash;
}

static Map_Node_t *mapCreateNode(const char *key, const void *value, const int valueSize) {
    const unsigned int keySize = strlen(key) + 1;
    const unsigned int valueOffset = keySize + (sizeof(void *) - keySize) % sizeof(void *); /* Align according to the number of bits in the system */
    Map_Node_t *node = TOOL_MALLOC(sizeof(*node) + valueOffset + valueSize);
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
        TOOL_FREE(map->buckets);
    }
    Map_Node_t **buckets = TOOL_MALLOC(sizeof(&map->buckets) * bucketNumber);
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
        case MAP_TYPE_QUIC_SERVER_CONN:
        case MAP_TYPE_QUIC_SEND_STREAM:
        case MAP_TYPE_QUIC_READ_STREAM:{instance->typeSize = size;break;}
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

void *mapSet(Map_t *map, const char *key, void *value, uint16_t valueSize) {
    int number, err;
    Map_Node_t **next, *node;
    if(valueSize == 0) valueSize = map->typeSize; /* In addition to copying the value in the address, you need to pass the data length, and other times you can pass 0 */
    /* If the exact key already exists, replace the value */
    next = mapGetNodeRef(&map->mapBase, key);
    if(next) {
        if(map->isCpyAddr) memcpy((*next)->value, value, valueSize);
        else memcpy((*next)->value, &value, map->typeSize);
        return (*next)->value;
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
        if(node) TOOL_FREE(node);
        return NULL;
    }

    mapAddNode(&map->mapBase, node);
    map->mapBase.nodeNumber++;

    return node->value;
}

void mapRemove_(Map_Base_t *map, const char *key) {
    Map_Node_t *node;
    Map_Node_t **next = mapGetNodeRef(map, key);
    if(next) {
        node = *next;
        *next = (*next)->next;
        TOOL_FREE(node);
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
            TOOL_FREE(node);
            node = next;
        }
    }
    TOOL_FREE(map->buckets);
}

/* MEMORY BLOCK */
DataBlock_t *dataBlockInit(const uint32_t capacity) {
    DataBlock_t *block = TOOL_MALLOC(sizeof(DataBlock_t));
    if(block == NULL) {
        DEBUG_PRINT("In quicTools, dataBlockInit: out of memory\n");
        return NULL;
    }
    block->data = TOOL_MALLOC(capacity);
    block->offset = 0;
    block->length = 0;
    block->capacity = capacity;
    block->next = NULL;
    block->isFin = false;

    return block;
}

void dataBlockClear(DataBlock_t *block) {
    if(block) {
        free(block->data);
        free(block);
    }
}

/* RBTree */
#define rb_parent(r) ((r)->parent)
#define rb_color(r) ((r)->color)
#define rb_is_red(r) (rb_color(r) == RED)
#define rb_is_black(r) (rb_color(r) == BLACK)
#define rb_set_black(r) do { (r)->color = BLACK; } while(0)
#define rb_set_red(r) do { (r)->color = RED; } while(0)
#define rb_set_parent(r, p) do { (r)->parent = p; } while(0)
#define rb_set_color(r, c) do { (r)->color = c; } while(0)

/* Create a new RBTree and return its root */
RBRoot_t* createRBTree(void) {
    RBRoot_t *root = TOOL_MALLOC(sizeof(RBRoot_t));
    if(root == NULL) {
        DEBUG_PRINT("In quicTools, createRBTree: out of memory\n");
        return NULL;
    }
    root->node = NULL;
    root->size = 0;
    root->externResourcePtr = NULL;

    return root;
}
/* Search function */
static RBNode_t* search(RBNode_t *node, const KeyType_t key) {
    while (node != NULL && node->key != key) {
        if (key < node->key) node = node->left;
        else node = node->right;
    }
    return node;
}
/* Search a node in RBTree with iteration method */
RBNode_t* searchRBTree(const RBRoot_t *root, const KeyType_t key) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, searchRBTree: root is NULL\n");
        return NULL;
    }
    return search(root->node, key);
}
/* Find minimum function */
static RBNode_t* minimum(RBNode_t *node) {
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, minimum: node is NULL\n");
        return NULL;
    }
    while (node->left != NULL) node = node->left;
    return node;
}
/* Find the minimum node in RBTree
 * @param root: the root of RBTree
 * @param data: the data's pointer's pointer of the minimum node
 * return: 0 success, -1 root is NULL, 1 node is NULL
 */
int RBTreeMinimum(const RBRoot_t *root, DataPtr_t *dataPtrPtr) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, RBTreeMinimum: root is NULL\n");
        return -1;
    }
    const RBNode_t *node = minimum(root->node);
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, RBTreeMinimum: node is NULL\n");
        return 1;
    }
    *dataPtrPtr = node->data;
    return 0;
}
/* Find maximum function */
static RBNode_t* maximum(RBNode_t *node) {
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, maximum: node is NULL\n");
        return NULL;
    }
    while (node->right != NULL) node = node->right;
    return node;
}
/* Find the maximum node in RBTree
 * @param root: the root of RBTree
 * @param data: the data's pointer's pointer of the maximum node
 */
int RBTreeMaximum(const RBRoot_t *root, DataPtr_t *dataPtrPtr) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, RBTreeMaximum: root is NULL\n");
        return -1;
    }
    const RBNode_t *node = maximum(root->node);
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, RBTreeMaximum: node is NULL\n");
        return -1;
    }
    *dataPtrPtr = node->data;
    return 0;
}
/* Rotate the node of the RBTree to the left */
static int leftRotate(RBRoot_t *root, RBNode_t *node) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, leftRotate: root is NULL\n");
        return -1;
    }
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, leftRotate: node is NULL\n");
        return -1;
    }
    RBNode_t *right = node->right;
    if (right == NULL) {
        DEBUG_PRINT("In quicTools, leftRotate: right is NULL\n");
        return 0;
    }
    node->right = right->left;
    if (right->left != NULL) right->left->parent = node;
    right->parent = node->parent;
    if (node->parent == NULL) root->node = right;
    else if (node == node->parent->left) node->parent->left = right;
    else node->parent->right = right;
    right->left = node;
    node->parent = right;

    return 0;
}
/* Rotate the node of the RBTree to the right */
static int rightRotate(RBRoot_t *root, RBNode_t *node) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, rightRotate: root is NULL\n");
        return -1;
    }
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, rightRotate: node is NULL\n");
        return -1;
    }
    RBNode_t *left = node->left;
    if (left == NULL) {
        DEBUG_PRINT("In quicTools, rightRotate: left is NULL\n");
        return 0;
    }
    node->left = left->right;
    if (left->right != NULL) left->right->parent = node;
    left->parent = node->parent;
    if (node->parent == NULL) root->node = left;
    else if (node == node->parent->right) node->parent->right = left;
    else node->parent->left = left;
    left->right = node;
    node->parent = left;

    return 0;
}
/* Insert function */
/* After insert, use this function, let RBTree be balance */
static int insertRBTreeFixup(RBRoot_t *root, RBNode_t *node) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, insertRBTreeFixup: root is NULL\n");
        return -1;
    }
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, insertRBTreeFixup: node is NULL\n");
        return -1;
    }
    RBNode_t *parent; // parent node
    while ((parent = rb_parent(node)) != NULL && rb_is_red(parent)) { // parent node exist and is red
        RBNode_t *gParent = rb_parent(parent); // grandparent node
        if (parent == gParent->left) { // parent node is left child of grandparent node
            { // case 1: uncle node is red
                RBNode_t *uncle = gParent->right;
                if (uncle && rb_is_red(uncle)) {
                    rb_set_black(uncle);
                    rb_set_black(parent);
                    rb_set_red(gParent);
                    node = gParent;
                    continue;
                }
            }
            if (parent->right == node) { // case 2: uncle node is black and node is right child
                leftRotate(root, parent);
                RBNode_t *tmp = parent;
                parent = node;
                node = tmp;
            }
            rb_set_black(parent);
            rb_set_red(gParent);
            rightRotate(root, gParent);
        }
        else { // parent node is right child of grandparent node
            { // case 1: uncle node is red
                RBNode_t *uncle = gParent->left;
                if (uncle && rb_is_red(uncle)) {
                    rb_set_black(uncle);
                    rb_set_black(parent);
                    rb_set_red(gParent);
                    node = gParent;
                    continue;
                }
            }
            if (parent->left == node) { // case 2: uncle node is black and node is left child
                rightRotate(root, parent);
                RBNode_t *tmp = parent;
                parent = node;
                node = tmp;
            }
            // case 3: uncle node is black and node is right child
            rb_set_black(parent);
            rb_set_red(gParent);
            leftRotate(root, gParent);
        }
    }
    // let root node be black
    rb_set_black(root->node);

    return 0;
}

/* Insert a node to RBTree */
static void insertRBTreeNode(RBRoot_t *root, RBNode_t *node) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, insertRBTree: root is NULL\n");
        return;
    }
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, insertRBTree: node is NULL\n");
        return;
    }
    RBNode_t *parentNode = NULL;
    RBNode_t *currNode = root->node;
    while (currNode != NULL) {
        parentNode = currNode;
        if (node->key < currNode->key) currNode = currNode->left;
        else currNode = currNode->right;
    }
    rb_parent(node) = parentNode;
    if (parentNode != NULL) {
        if (node->key < parentNode->key) parentNode->left = node;
        else parentNode->right = node;
    }
    else root->node = node;
    node->color = RED;
    insertRBTreeFixup(root, node);
}

/* Create a new RBTree node */
RBNode_t* createRBTreeNode(const KeyType_t key, DataPtr_t data, const DataSizeType_t dataSize, RBNode_t *parentNode, RBNode_t *leftNode, RBNode_t *rightNode) {
    RBNode_t *node = TOOL_MALLOC(sizeof(RBNode_t) + dataSize);
    if(node == NULL) {
        DEBUG_PRINT("In quicTools, createRBTreeNode: out of memory\n");
        return NULL;
    }
    void *dataPtr = node + (int)sizeof(RBNode_t); // TODO: may debug
    memcpy(dataPtr, data, dataSize);

    node->key = key;
    node->size = dataSize;
    node->data = dataPtr;
    node->left = leftNode;
    node->right = rightNode;
    node->parent = parentNode;
    node->color = BLACK;

    return node;
}
/* Init a node and insert it to RBTree */
int insertRBTree(RBRoot_t *root, const KeyType_t key, DataPtr_t data, const DataSizeType_t dataSize) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, insertRBTree: root is NULL\n");
        return -1;
    }
    if (search(root->node, key) != NULL) {
        DEBUG_PRINT("In quicTools, insertRBTree: key is already exist\n");
        return -1;
    }
    RBNode_t *node = createRBTreeNode(key, data, dataSize, NULL, NULL, NULL);
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, insertRBTree: createRBTreeNode failed\n");
        return -1;
    }
    insertRBTreeNode(root, node);
    return 0;
}
/* Delete function */
/* After delete, use this function, let RBTree be balance */
static void deleteRBTreeFixup(RBRoot_t *root, RBNode_t *node, RBNode_t *parent) {
    RBNode_t *other;
    while ((!node || rb_is_black(node)) && node != root->node) {
        if (parent->left == node) {
            other = parent->right;
            if (rb_is_red(other)) {
                rb_set_black(other);
                rb_set_red(parent);
                leftRotate(root, parent);
                other = parent->right;
            }
            if ((!other->left || rb_is_black(other->left)) &&
                (!other->right || rb_is_black(other->right))) {
                rb_set_red(other);
                node = parent;
                parent = rb_parent(node);
            }
            else {
                if (!other->right || rb_is_black(other->right)) {
                    rb_set_black(other->left);
                    rb_set_red(other);
                    rightRotate(root, other);
                    other = parent->right;
                }
                rb_set_color(other, rb_color(parent));
                rb_set_black(parent);
                rb_set_black(other->right);
                leftRotate(root, parent);
                node = root->node;
                break;
            }
        }
        else {
            other = parent->left;
            if (rb_is_red(other)) {
                rb_set_black(other);
                rb_set_red(parent);
                rightRotate(root, parent);
                other = parent->left;
            }
            if ((!other->left || rb_is_black(other->left)) &&
                (!other->right || rb_is_black(other->right))) {
                rb_set_red(other);
                node = parent;
                parent = rb_parent(node);
            }
            else {
                if (!other->left || rb_is_black(other->left)) {
                    rb_set_black(other->right);
                    rb_set_red(other);
                    leftRotate(root, other);
                    other = parent->left;
                }
                rb_set_color(other, rb_color(parent));
                rb_set_black(parent);
                rb_set_black(other->left);
                rightRotate(root, parent);
                node = root->node;
                break;
            }
        }
    }
    if (node) rb_set_black(node);
}
/* Delete a node from RBTree with node */
static void deleteRBTreeNode(RBRoot_t *root, RBNode_t *node) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, deleteRBTreeNode: root is NULL\n");
        return;
    }
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, deleteRBTreeNode: node is NULL\n");
        return;
    }
    RBNode_t *child, *parent;
    int color;
    if (node->left && node->right) {
        RBNode_t *replace = node;
        replace = replace->right;
        while (replace->left != NULL) replace = replace->left;
        if (rb_parent(node)) {
            if (rb_parent(node)->left == node) rb_parent(node)->left = replace;
            else rb_parent(node)->right = replace;
        }
        else root->node = replace;
        child = replace->right;
        parent = rb_parent(replace);
        color = rb_color(replace);
        if (parent == node) parent = replace;
        else {
            if (child) rb_set_parent(child, parent);
            parent->left = child;
            replace->right = node->right;
            rb_set_parent(node->right, replace);
        }
        replace->parent = node->parent;
        replace->color = node->color;
        replace->left = node->left;
        node->left->parent = replace;
        if (color == BLACK) deleteRBTreeFixup(root, child, parent);
        TOOL_FREE(node);
        return;
    }
    if (node->left != NULL) child = node->left;
    else child = node->right;
    parent = node->parent;
    color = node->color;
    if (child) child->parent = parent;
    if (parent) {
        if (parent->left == node) parent->left = child;
        else parent->right = child;
    }
    else root->node = child;
    if (color == BLACK) deleteRBTreeFixup(root, child, parent);
    TOOL_FREE(node);
}
/* Delete a node from RBTree with key */
int deleteRBTree(RBRoot_t *root, const KeyType_t key) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, deleteRBTree: root is NULL\n");
        return -1;
    }
    RBNode_t *node = search(root->node, key);
    if (node == NULL) {
        DEBUG_PRINT("In quicTools, deleteRBTree: key is not exist\n");
        return -1;
    }
    deleteRBTreeNode(root, node);
    return 0;
}

/* Clear function */
static void clearRBTreeNode(RBNode_t *node) {
    if (node == NULL) return;
    if (node->left != NULL) clearRBTreeNode(node->left);
    if (node->right != NULL) clearRBTreeNode(node->right);
    TOOL_FREE(node);
}
/* Clear the RBTree */
void clearRBTree(RBRoot_t *root) {
    if (root == NULL) {
        DEBUG_PRINT("In quicTools, clearRBTree: root is NULL\n");
        return;
    }
    RBNode_t *node = root->node;
    root->size = 0;
    root->externResourcePtr = NULL; // users manage external resources
    clearRBTreeNode(node);
    TOOL_FREE(root);
}