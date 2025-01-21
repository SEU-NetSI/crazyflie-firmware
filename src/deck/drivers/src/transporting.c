#include "transporting.h"

void transportInit(void)
{

}

int uwbConnect(const UWB_Address_t peer)
{
    quicClientSendConnRequest(peer);
    return DWT_SUCCESS;
}