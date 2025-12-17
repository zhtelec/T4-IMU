/*
 * Copyright (c) 2018 zhtlab
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#define _FIFO_C_

#include	<stdint.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>

#include	"fifo.h"

#define	minvalue(a,b)		(a < b ? a : b)

char const FIFO_C_RCSid[] = "$Id: fifo.c,v 1.5 2005/06/28 06:07:14 buggy Exp $";



static ring_fifo_t	fifo[FIFO_ENTRY];

/*
 * FifoInit()
 * input:  none
 * output: none
 *
 * May  8, 2000 buggy
 *    1st edition
 */
void
FifoInit(void)
{
#if 0
  int		i;
  ring_fifo_t	*ptr;

  ptr = fifo;
  for(i = 0; i < FIFO_ENTRY; i++, ptr++) {
    ptr->buf = (char *)NULL;
    ptr->flag = 0;
  }
#endif
  memset(&fifo, 0, sizeof(fifo));

  return;
}


/*
 * int FifoCreate(int sizeExp)
 * int FifoCreateWithBuf(int sizeExp, void *buf)
 * input:  int size  buffer size
 *                   size must be log2(real size) ex. 10 -> 1024, 12 -> 4096
 * output: int FifoCreate  fifo entry number
 *
 * May  8, 2000 buggy
 *    1st edition
 * Jan 27, 2004 buggy
 *    FifoCreateWithBuf() is added
 */
int
FifoCreate(int sizeExp)
{
  int		result = FIFO_ERRNO_UNKNOWN;
  int		i;
  int		sizepow;
  ring_fifo_t	*ptr;

  if(sizeExp > FIFO_SIZEEXP_MAX) {
    result = FIFO_ERRNO_PARAMEXCEEDED;
    goto end;
  }

  ptr = fifo;
  for(i = 0; i < FIFO_ENTRY && ptr->buf; i++, ptr++);

  result = FIFO_ERRNO_NOSPACE;
  if(i < FIFO_ENTRY) {
    sizepow = 1 << sizeExp;
    if((ptr->buf = (char *) malloc(sizepow)) != NULL) {
      ptr->start = 0;
      ptr->end   = 0;
      ptr->size  = sizepow;

      ptr->flag &= ~FIFO_EXTERNAL_BUFFER;

      result = i;
    }
  }

end:
  return result;	/* return with fifo entry */
}

int
FifoCreateWithBuf(int sizeExp, void *buf)
{
  int		result = -1;
  int		i;
  int		sizepow;
  ring_fifo_t	*ptr;

  if(sizeExp > FIFO_SIZEEXP_MAX) {
    result = FIFO_ERRNO_PARAMEXCEEDED;
    goto end;
  }

  ptr = fifo;
  for(i = 0; i < FIFO_ENTRY && ptr->buf; i++, ptr++);

  result = FIFO_ERRNO_NOSPACE;
  if(i != FIFO_ENTRY) {
    sizepow = 1 << sizeExp;
    ptr->buf = (char *)buf;
    ptr->start = 0;
    ptr->end   = 0;
    ptr->size  = sizepow;

    ptr->flag |= FIFO_EXTERNAL_BUFFER;

    result = i;
  }

end:
  return result;	/* return with fifo entry */
}


/*
 * int FifoCreateInt(int size)
 * int FifoCreateIntWithBuf(int size, void *buf)
 * input:  int size  buffer size
 * output: int FifoCreate  fifo entry number
 *
 * Jun 11, 2005 buggy
 *    1st edition
 */
int
FifoCreateInt(int size)
{
  int		result = -1;
  int		i;
  ring_fifo_t	*ptr;

  if(size > FIFO_SIZE_MAX) {
    result = FIFO_ERRNO_PARAMEXCEEDED;
    goto end;
  }

  result = FIFO_ERRNO_NOSPACE;
  ptr = fifo;
  for(i = 0; i < FIFO_ENTRY && ptr->buf; i++, ptr++);

  if(i != FIFO_ENTRY) {
    if((ptr->buf = (char *) malloc(size)) != NULL) {
      ptr->start = 0;
      ptr->end   = 0;
      ptr->size  = size;

      ptr->flag &= ~FIFO_EXTERNAL_BUFFER;

      result = i;
    }
  }

end:
  return result;	/* return with fifo entry */
}

int
FifoCreateIntWithBuf(int size, void *buf)
{
  int		result = -1;
  int		i;
  ring_fifo_t	*ptr;

  if(size > FIFO_SIZE_MAX) {
    result = FIFO_ERRNO_PARAMEXCEEDED;
    goto end;
  }

  ptr = fifo;
  for(i = 0; i < FIFO_ENTRY && ptr->buf; i++, ptr++);

  result = FIFO_ERRNO_NOSPACE;
  if(i != FIFO_ENTRY) {
    ptr->buf = (char *)buf;
    ptr->start = 0;
    ptr->end   = 0;
    ptr->size  = size;

    ptr->flag |= FIFO_EXTERNAL_BUFFER;

    result = i;
  }

end:
  return result;	/* return with fifo entry */
}


/*
 * FifoDestroy()
 * input:  int no  number of fifo buffer destory
 * output: none
 *
 * May  8, 2000 buggy
 *    1st edition
 */
void
FifoDestroy(int no)
{
  if((fifo + no)->buf) {
    if(!((fifo + no)->flag & FIFO_EXTERNAL_BUFFER)) {
      free((fifo + no)->buf);
    }
    (fifo + no)->buf = (char *)NULL;
  }

  return;
}


/*
 * FifoClear()
 * input:  int no      number of fifo buffer write
 * output: none
 *
 * Dec  8, 2000 hmura
 *    1st edition
 */
void
FifoClear(int no)
{
  if((fifo + no)->buf) {
    (fifo + no)->start = 0;
    (fifo + no)->end   = 0;
  }
}


/*
 * FifoWriteIn()
 * input:  int no      number of fifo buffer write
 *         char *buf   data buffer
 *         u int size  data size
 * output: really write data size
 *
 * May  8, 2000 buggy
 *    1st edition
 */
int
FifoWriteIn(int no, char *buf, unsigned int size)
{
  int			fsz = 0, fsz1, fsz2;
  ring_fifo_t		*ptr;

  /* fifo no's fifo pointer */
  ptr = fifo + no;

  if(!ptr->buf) goto fail;

  /* get free buffer size of fifo*/
  fsz = ptr->end - ptr->start - 1;
  if(fsz < 0) {
    fsz += ptr->size;
  }
  fsz2 = fsz = minvalue(fsz, size);

  /* if the buffer boundary is over, transfat must be divideed */
  fsz1 = 0;
  if(ptr->start + fsz > ptr->size) {
    fsz1 = ptr->size - ptr->start;
    memcpy(ptr->buf + ptr->start, buf, fsz1);
    fsz2 -= fsz1;
    ptr->start = 0;
  }

  /* 1st copy, if not to  divided  or 2nd copy, if needs to divided */
  memcpy(ptr->buf + ptr->start, buf + fsz1, fsz2);

  /* change the start pointer */
  ptr->start = ptr->start + fsz2;
  if(ptr->start >= ptr->size) {
    ptr->start -= ptr->size;
  }

fail:
  return fsz;
}


/*
 * FifoReadOut()
 * input:  int no      number of fifo buffer read
 *         char *buf   data buffer
 *         u int size  data size
 * output: really write data size
 *
 * May  8, 2000 buggy
 *    1st edition
 */
int
FifoReadOut(int no, char *buf, unsigned int size)
{
  int			fsz=0, fsz1, fsz2;
  ring_fifo_t		*ptr;

  /* fifo no's fifo pointer */
  ptr = fifo + no;

  if(!ptr->buf) goto fail;

  /* get free buffer size of fifo*/
  fsz = ptr->start - ptr->end;
  if(fsz) {
    if(fsz < 0) {
      fsz += ptr->size;
    }
    fsz2 = fsz = minvalue(fsz, size);

    /* if the buffer boundary is over, transfat must be divideed */
    fsz1 = 0;
    if(ptr->end + fsz > ptr->size) {
      fsz1 = ptr->size - ptr->end;
      memcpy(buf, ptr->buf + ptr->end, fsz1);
      fsz2 -= fsz1;
      ptr->end = 0;
    }

    memcpy(buf + fsz1, ptr->buf + ptr->end, fsz2);

    /* change the end pointer */
    ptr->end = ptr->end + fsz2;
    if(ptr->end >= ptr->size) {
      ptr->end -= ptr->size;
    }
  }

fail:
  return fsz;
}


/*
 * int FifoGetWritePointer(int no, char **buf, int *size)
 * input:  int no      number of fifo buffer write
 *         char **buf  data buffer
 *         int  *size  data size
 * output: SUCCESS or INVALIDARG
 *
 * Jun 10, 2005 buggy
 *    1st edition
 */
int
FifoGetWritePointer(int no, char **buf, int *size)
{
  int		result = FIFO_ERRNO_INVALIDARG;
  int		sz;

  ring_fifo_t	*p;

  p = &fifo[no];

  if(!p->buf) goto fail;

  if(buf && size) {
    sz = p->end - p->start - 1;
    if(sz < 0) {
      sz = p->size - p->start;
    }

    *buf = p->buf + p->start;
    *size = sz;

    result = FIFO_ERRNO_SUCCESS;
  }

fail:
  return result;
}


/*
 * int FifoGetWritePointer(int no, char **buf, int *size)
 * input:  int no      number of fifo buffer write
 *         char **buf  data buffer
 *         int  *size  data size
 * output: SUCCESS or INVALIDARG
 *
 * Jun 10, 2005 buggy
 *    1st edition
 */
int
FifoGetReadPointer(int no, char **buf, int *size)
{
  int		result = FIFO_ERRNO_INVALIDARG;
  int		sz;

  ring_fifo_t	*p;

  p = &fifo[no];

  if(!p->buf) goto fail;

  if(buf && size) {
    sz = p->start - p->end;
    if(sz < 0) {
      sz = p->size - p->end;
    }

    *buf = p->buf + p->end;
    *size = sz;

    result = FIFO_ERRNO_SUCCESS;
  }

fail:
  return result;
}


/*
 * int FifoAddWritePointer(int no, int size)
 * input:  int no      number of fifo buffer write
 *         int size    add size;
 * output: SUCCESS or PARAMEXCEEDED
 *
 * Jun 10, 2005 buggy
 *    1st edition
 */
int
FifoAddWritePointer(int no, int size)
{
  int			result = FIFO_ERRNO_PARAMEXCEEDED;

  ring_fifo_t	*p;

  p = &fifo[no];

  if(!p->buf) goto fail;

  if(size >= 0 && size < p->size) {
    if(FifoGetEmptyLen(no) >= size) {
      p->start += size;
      if(p->start >= p->size) {
	p->start -= p->size;	
      }

#if 0
      printf("WWW %d %d %d %d\n",
	     p->size, p->start, p->end,
	     FifoGetEmptyLen(no));
#endif
      result = FIFO_ERRNO_SUCCESS;
    }
  }

fail:
  return result;
}


/*
 * int FifoGetWritePointer(int no, char **buf, int *size)
 * input:  int no      number of fifo buffer write
 *         char *buf   data buffer
 *         u int size  data size
 * output: really write data size
 *
 * Jun 10, 2005 buggy
 *    1st edition
 */
int
FifoAddReadPointer(int no, int size)
{
  int			result = FIFO_ERRNO_PARAMEXCEEDED;

  ring_fifo_t	*p;

  p = &fifo[no];

  if(!p->buf) goto fail;

  if(size >= 0 && size < p->size) {
    if(FifoGetDirtyLen(no) >= size) {
      p->end += size;
      if(p->end >= p->size) {
	p->end -= p->size;
      }
    }

#if 0
    printf("RRR %d %d %d %d\n",
	   p->size, p->start, p->end,
	   FifoGetEmptyLen(no));
#endif

    result = FIFO_ERRNO_SUCCESS;
  }

fail:
  return result;
}


/*
 * int FifoGetDirtyLen(int no)
 * input:  int no      number of fifo buffer write
 *         char *buf   data buffer
 *         u int size  data size
 * output: really write data size
 *
 * Jun 10, 2005 buggy
 *    1st edition
 */
int
FifoGetDirtyLen(int no)
{
  int		size = 0;

  if(!fifo[no].buf) goto fail;

  size = (fifo + no)->start - (fifo + no)->end;
  if(size < 0) {
    size += (fifo + no)->size;
  }

fail:
  return size;
}


/*
 * int FifoGetEmptyLen(int no)
 * input:  int no      number of fifo buffer write
 *         char *buf   data buffer
 *         u int size  data size
 * output: really write data size
 *
 * Jun 10, 2005 buggy
 *    1st edition
 */
int
FifoGetEmptyLen(int no)
{
  int		size = 0;

  if(!fifo[no].buf) goto fail;

  size = (fifo + no)->end - (fifo + no)->start - 1;
  if(size < 0) {
    size += (fifo + no)->size;
  }

fail:
  return size;
}
