
#ifndef __AES_H__
#define __AES_H__

#include <stdint.h>
#include <stdbool.h>
#include "syslink.h"

struct crtpLinkOperations aeslinkOp;

void aeslinkInit();
bool aeslinkTest();
void aesEnableTunnel();
struct crtpLinkOperations * aeslinkGetLink();
int aeslinkSendCRTPPacket(CRTPPacket *p);


//struct crtpLinkOperations aeslinkGetLink();
#endif //__AES_H__
