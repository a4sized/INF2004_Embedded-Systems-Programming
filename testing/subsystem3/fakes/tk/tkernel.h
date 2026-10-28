#ifndef IR_SNAPSHOT_TEST_TKERNEL_H
#define IR_SNAPSHOT_TEST_TKERNEL_H

#include <stddef.h>

typedef int ID;
typedef int ER;
typedef struct
{
    int mtxatr;
} T_CMTX;

#define E_OK 0
#define TA_TFIFO 1
#define TA_INHERIT 2
#define TMO_FEVR (-1)

ID tk_cre_mtx (T_CMTX const * p_config);
ER tk_loc_mtx (ID mutex, int timeout);
ER tk_unl_mtx (ID mutex);

#endif
