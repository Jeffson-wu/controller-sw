#include "util.h"
#include "string.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"



/*Util functions*/

static void reverse(char s[])
{
 int i, j;
 char c;

 for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
     c = s[i];
     s[i] = s[j];
     s[j] = c;
 }
}

void Itoa(int n, char s[])
{
  int i, sign;

  if ((sign = n) < 0)  
    n = -n;
  i = 0;
  do {
    s[i++] = n % 10 + '0';
  } while ((n /= 10) > 0);
  if (sign < 0)
    s[i++] = '-';
  s[i] = '\0';
  reverse(s);
}

int addStrToBuf(char *buffer, const char *str, int maxsize)
{
  if(strlen(buffer)+strlen(str)>maxsize)
  {
    configASSERT(pdFALSE);
    return -1;
  }
  else
  {
    strcat(buffer,str);
  }
  return 0;
}



