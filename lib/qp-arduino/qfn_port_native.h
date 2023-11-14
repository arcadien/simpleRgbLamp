#ifndef QFN_PORT_H
#error use #include "qfn_port.h"
#endif

#include <stdio.h>
#include <stdint.h>      /* Exact-width types. WG14/N843 C99 Standard */
#include <stdbool.h>     /* Boolean type.      WG14/N843 C99 Standard */

#define Q_NORETURN __attribute__((noreturn)) void

#define LOG(message) printf("%s", message);

/* QF-nano interrupt disable/enable... */
#define QF_INT_DISABLE()
#define QF_INT_ENABLE()

/* QF-nano interrupt disabling policy for interrupt level */
/*#define QF_ISR_NEST*/ /* nesting of ISRs not allowed */

#define QV_CPU_SLEEP() printf("QV_CPU_SLEEP()\n")
#define QF_RESET()     printf("QF_RESET()\n")

#include "qepn.h"        /* QEP-nano platform-independent public interface */
#include "qfn.h"         /* QF-nano  platform-independent public interface */
#include "qvn.h"         /* QV-nano  platform-independent public interface */