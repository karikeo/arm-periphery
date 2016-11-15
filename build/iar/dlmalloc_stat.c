
#include <iar_dlmalloc.h>
#include <stdio.h>

/* Extern declaration */
extern void  __iar_dlmalloc_stats(void);


/* Internal declarations */
struct internal_malloc_stat_struct {
  size_t maxfp;
  size_t fp;
  size_t used;
};
void __iar_internal_malloc_stats(struct internal_malloc_stat_struct *);

void __iar_dlmalloc_stats() {
  struct internal_malloc_stat_struct imss;
  __iar_internal_malloc_stats(&imss);
#if !_DLIB_FILE_DESCRIPTOR
    printf("max system bytes = %10lu\n", (unsigned long)(imss.maxfp));
    printf("system bytes     = %10lu\n", (unsigned long)(imss.fp));
    printf("in use bytes     = %10lu\n", (unsigned long)(imss.used));
#else 
    fprintf(stderr, "max system bytes = %10lu\n", (unsigned long)(imss.maxfp));
    fprintf(stderr, "system bytes     = %10lu\n", (unsigned long)(imss.fp));
    fprintf(stderr, "in use bytes     = %10lu\n", (unsigned long)(imss.used));
#endif
}
