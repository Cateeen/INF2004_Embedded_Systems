#ifndef IMU_I2C_HOST_TKERNEL_H
#define IMU_I2C_HOST_TKERNEL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef int32_t ER;
typedef int32_t ID;
typedef int32_t W;
typedef uint32_t UW;
typedef int32_t SZ;
typedef uint8_t UB;
typedef uint32_t RELTIM;

typedef struct {
	UW hi;
	UW lo;
} SYSTIM;

#define IMPORT extern
#define EXPORT extern
#define E_OK       0
#define E_PAR     -17
#define E_OBJ     -41
#define E_IO      -57
#define E_TMOUT   -50
#define TD_READ    0x0001U
#define TD_WRITE   0x0002U
#define TD_UPDATE  0x0003U
#define TD_EXCL    0x0100U

IMPORT ID tk_opn_dev(const UB *devnm, uint32_t omode);
IMPORT ER tk_cls_dev(ID dd, uint32_t option);
IMPORT ER tk_swri_dev(ID dd, W start, const void *buf, SZ size, SZ *asize);
IMPORT ER tk_get_otm(SYSTIM *pk_tim);
IMPORT ER tk_dly_tsk(RELTIM dlytim);

#endif
