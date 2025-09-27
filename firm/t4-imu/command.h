#ifndef _COMMAND_H_
#define _COMMAND_H_


#define	COMMAND_ERRNO_SUCCESS           0
#define	COMMAND_ERRNO_UNKNOWN           (-1)


// debug
#define DEBUG_XXX_POS                0
#define DEBUG_XXX_MASK               (1 << (DEBUG_XXX_POS))
#define DEBUG_XXX_NO                 (0 << (DEBUG_XXX_POS))
#define DEBUG_XXX_YES                (1 << (DEBUG_XXX_POS))


struct _stCommand {
#define COMMAND_RECV_BUFFER_SIZE        128
  unsigned char         bufRecv[COMMAND_RECV_BUFFER_SIZE];
  int                   posRecv;
  
#define	COMMAND_RESPONSE_BUFFER_SIZE    128
  unsigned char         bufResponse[COMMAND_RESPONSE_BUFFER_SIZE];
};


void                    CommandInit(void);
void                    CommandLoop(void);
int                     CommandLoopRead(int unit);
void                    CommandClearParse(int unit);
void                    CommandGetParse(int unit, int *pac, char *(*pav[]));
int                     CommandGetch(int unit);


int                     CommandParser(int unit, char *p, int len);


#ifdef  _COMMAND_C_
static int              CommandDelim(char *av[], char *p, int len);

static int              CommandExecVersion(int unit, int ac, char *av[]);
//static int              CommandExecTimestamp(int unit, int ac, char *av[]);

static void             CommandExec(int unit, int ac, char *av[]);
//static void             CommandBuildVersionText(uint32_t *buf, uint8_t *str);

static int              CommandDebug(int unit, int ac, char *av[]);
#endif


#endif  /* _COMMAND_H_ */
