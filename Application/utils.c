#include "stm32f10x.h"
#include "RTOS.h"
#include "BSP.h"
#include "stdio.h"
#include "log.h"
#if defined LCD
#include "lcd_2x16.h"
#endif
#include "utils.h"

void short_date(char const *date, char *p)
{ 
    char buf[ 16 ];
    int month, day, year;
    static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    sscanf(date, "%s %d %d", buf, &day, &year);
    month = (strstr(month_names, buf)-month_names)/3+1;
    sprintf( p, "%d%02d%02d", year, month, day);
}

void build_dt(char* p)
{
  char buf[ 16 ];
  int hour, min, sec;  
  short_date( __DATE__, buf );
  sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec);  
  sprintf( p, "%s%02d%02d%02d",buf, hour, min, sec);
}
