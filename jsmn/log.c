/* log: stderr logging information. */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "log.h"

#if 0
void log_die(char *msg, ...)
{
    va_list argp;

    log_null(msg);

    va_start(argp, msg);
    vfprintf(stderr, msg, argp);
    va_end(argp);

    fprintf(stderr, "\n");
    abort();
}
#else
void log_die(char *msg, ...)
{
  char out1[300]; /*buffer for debug printf*/
  va_list argp;

  va_start(argp, msg);
  sprintf(out1, msg, argp);  gdi_send_msg_response(out1);
  va_end(argp);
}
#endif

void log_info(char *msg, ...)
{
    va_list argp;

    log_null(msg);

    va_start(argp, msg);
    vfprintf(stderr, msg, argp);
    va_end(argp);

    fprintf(stderr, "\n");
}
