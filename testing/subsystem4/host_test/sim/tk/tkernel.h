/* PC stand-in for <tk/tkernel.h>: just enough for subsystem 4's sources. */
#ifndef SIM_TKERNEL_H
#define SIM_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int32_t INT; typedef uint32_t UINT; typedef int32_t W; typedef uint32_t UW;
typedef int16_t H; typedef uint16_t UH; typedef uint8_t UB; typedef int32_t BOOL;
typedef int32_t ID; typedef int32_t ER; typedef int32_t PRI; typedef int32_t SZ;
typedef uint32_t ATR; typedef int32_t TMO; typedef uint32_t RELTIM; typedef void (*FP)();
typedef struct { W hi; UW lo; } SYSTIM;
typedef struct t_msg { struct t_msg *next; } T_MSG;
#define TRUE 1
#define FALSE 0
#define EXPORT
#define IMPORT extern
#define LOCAL static
#define E_OK 0
#define E_NOSPT (-9)
#define E_OBJ (-41)
#define E_TMOUT (-50)
#define E_IO (-57)
#define TMO_POL 0
#define TMO_FEVR (-1)
#define TA_TFIFO 0x0U
#define TA_WMUL 0x8U
#define TA_INHERIT 0x2U
#define TWF_ORW 0x01U
#define TWF_CLR 0x10U
#define TWF_BITCLR 0x20U
#define TD_UPDATE 0x3U
#define DI(x) ((x)=0)
#define EI(x) ((void)(x))
typedef struct { void *exinf; ATR mbfatr; SZ bufsz; INT maxmsz; } T_CMBF;
typedef struct { void *exinf; ATR mtxatr; PRI ceilpri; } T_CMTX;
typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
ID tk_cre_mbf(const T_CMBF *p); ER tk_snd_mbf(ID id, const void *msg, INT sz, TMO tmo);
INT tk_rcv_mbf(ID id, void *msg, TMO tmo);
ID tk_cre_mtx(const T_CMTX *p); ER tk_loc_mtx(ID id, TMO tmo); ER tk_unl_mtx(ID id);
ID tk_cre_flg(const T_CFLG *p); ER tk_set_flg(ID id, UINT ptn);
ER tk_wai_flg(ID id, UINT ptn, UINT mode, UINT *out, TMO tmo);
ER tk_dly_tsk(RELTIM ms); ER tk_get_otm(SYSTIM *t);
ID tk_opn_dev(const UB *name, UINT mode); ER tk_cls_dev(ID dd, UINT opt);
ER tk_swri_dev(ID dd, INT dn, const void *buf, SZ sz, SZ *asz);
#endif
