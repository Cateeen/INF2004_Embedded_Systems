#ifndef I2C_HOST_KERNEL_H
#define I2C_HOST_KERNEL_H
#include <stddef.h>
#include <stdint.h>
typedef uint32_t UW;
typedef unsigned int UINT;
typedef int INT;
typedef int W;
typedef int SZ;
typedef int ID;
typedef int ER;
typedef int PRI;
typedef int TMO;
typedef int BOOL;
typedef uint8_t UB;
typedef struct { UINT flgatr; } T_CFLG;
typedef struct { UINT intatr; void (*inthdr)(UINT); } T_DINT;
#define LOCAL static
#define IMPORT extern
#define EXPORT
#define Inline static inline
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_IO (-57)
#define E_PAR (-17)
#define E_TMOUT (-50)
#define TA_TFIFO 0
#define TA_HLNG 0
#define TWF_ORW 1
#define TWF_BITCLR 2
#define TK_SUPPORT_SMP 0
extern UINT i2c_host_irq_masked;
#define DI(sts) do { (sts) = i2c_host_irq_masked; i2c_host_irq_masked = 1U; } while (0)
#define EI(sts) do { i2c_host_irq_masked = (sts); } while (0)
UW in_w(UW address);
void out_w(UW address, UW value);
void set_w(UW address, UW value);
void clr_w(UW address, UW value);
void ClearInt(UINT intno);
void EnableInt(UINT intno, PRI priority);
ID tk_cre_flg(const T_CFLG *config);
ER tk_del_flg(ID id);
ER tk_set_flg(ID id, UINT bits);
ER tk_clr_flg(ID id, UINT bits);
ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT *pattern, TMO timeout);
ER tk_def_int(UINT intno, const T_DINT *config);
#endif
