#ifndef _FIFO_H_
#define _FIFO_H_

#ifdef  _FIFO_C_

#endif

#define	FIFO_ERRNO_SUCCESS		0
#define	FIFO_ERRNO_UNKNOWN		(-1)	/* unknow fifo error */
#define	FIFO_ERRNO_NOMEMORY		(-2)	/* can't allocate memory */
#define	FIFO_ERRNO_NOSPACE		(-3)	/* no space in the system */
#define	FIFO_ERRNO_INVALIDARG		(-4)	/* invalid argument */
#define	FIFO_ERRNO_PARAMEXCEEDED	(-5)	/* parameter exceeded */
#define	FIFO_ERRNO_PERM			(-6)	/* operation not permitted */


#ifdef  _FIFO_C_

#define	FIFO_ENTRY		32
#define	FIFO_SIZEEXP_MAX	30
#define	FIFO_SIZE_MAX		(1 << 30)

typedef struct {
  char		*buf;
  uint32_t	flag;
#define	FIFO_UP				(1 << 0)        /* up */
#define	FIFO_EXTERNAL_BUFFER		(1 << 1)        /* WithBuffer() */
  int		start;
  int		end;
  int		size;
} ring_fifo_t;

#endif  /* _FIFO_H_ */

/*
 * prototype
 */
void		FifoInit(void);
int 		FifoCreate(int sizeExp);
int		FifoCreateWithBuf(int sizeExp, void *buf);
int 		FifoCreateInt(int size);
int		FifoCreateIntWithBuf(int size, void *buf);
void		FifoDestroy(int no);
void		FifoClear(int no);
int 		FifoWriteIn(int no, char *buf, unsigned int size);
int 		FifoReadOut(int no, char *buf, unsigned int size);

int		FifoGetWritePointer(int no, char **buf, int *size);
int		FifoGetReadPointer(int no, char **buf, int *size);
int		FifoAddWritePointer(int no, int size);
int		FifoAddReadPointer(int no, int size);

int 		FifoGetDirtyLen(int no);
int 		FifoGetEmptyLen(int no);


#endif  /* _FIFO_H_ */
