#include    <string.h>
#include    <math.h>
#include    <stdio.h>
#include    <stdlib.h>
#include    <epicsThread.h>
#include    <epicsString.h>
#include    <drvSup.h>
#include    <errlog.h>
#include    "motor.h"
#include    "drvLinMot.h"
#include    "asynOctetSyncIO.h"
#include    "epicsExport.h"

#define STATIC static
#define TIMEOUT 3.0         /* Command timeout in sec */
#define ASCII_0_TO_A 65     /* ASCII offset between 0 and A */
#define BUFF_SIZE 200       /* Maximum length of string to/from LinMot */

struct mess_queue
{
    struct mess_node *head;
    struct mess_node *tail;
};

/*----------------debugging-----------------*/
volatile int drvLinMotDebug = 0;
extern "C" {epicsExportAddress(int, drvLinMotDebug);}

static inline void Debug(int level, const char *format, ...) {
    if (level < drvLinMotDebug) {
      va_list pVar;
      va_start(pVar, format);
      vprintf(format, pVar);
      va_end(pVar);
    }
}

int LinMot_num_cards = 0;

/* Local data required for every driver; see "motordrvComCode.h" */
#include "motordrvComCode.h"

/*----------------functions-----------------*/
STATIC int recv_mess(int card, char *buff, int len);
STATIC RTN_STATUS send_mess(int card, const char *com, const char *name);
STATIC int send_recv_mess(int card, const char *out, char *in);
STATIC void start_status(int card);
STATIC int set_status(int card, int signal);
static long report(int level);
static long init();
STATIC int motor_init();
STATIC void query_done(int, int, struct mess_node *);

/*----------------functions-----------------*/
struct driver_table LinMot_access =
{
    motor_init,
    motor_send,
    motor_free,
    motor_card_info,
    motor_axis_info,
    &mess_queue,
    &queue_lock,
    &free_list,
    &freelist_lock,
    &motor_sem,
    &motor_state,
    &total_cards,
    &any_motor_in_motion,
    send_mess,
    recv_mess,
    set_status,
    query_done,
    start_status,
    &initialized,
    NULL
};

struct drvLinMot_drvet
{
    long number;
#ifdef __cplusplus
    long (*report) (int);
    long (*init) (void);
#else
    DRVSUPFUN report;
    DRVSUPFUN init;
#endif
} drvLinMot = {2, report, init};

extern "C" {epicsExportAddress(drvet, drvLinMot);}

STATIC struct thread_args targs = {SCAN_RATE, &LinMot_access, 0.0};

/*********************************************************
 * Print out driver status report
 *********************************************************/
static long report(int level)
{	
    if (LinMot_num_cards <=0) {
        epicsPrintf("LinMot report: No ontrollers found\n");
	}
    else {
        for (int card = 0; card < LinMot_num_cards; card++)
            if (motor_state[card])
                printf("LinMot controller %d active\n",
                    card);
    }
    return (OK);
}

static long init()
{	
    if (LinMot_num_cards <= 0)
    {
        Debug(1, "LinMotSetup() is missing from startup script.\n");
        return (ERROR);
    }
    return (OK);
}

STATIC void query_done(int card, int axis, struct mess_node *nodeptr)
{	
}

STATIC void start_status(int card)
{	
}


/**************************************************************
 * Query position and status for an axis
 **************************************************************/
STATIC int set_status(int card, int signal)
{	
    register struct mess_info *motor_info;
    char command[BUFF_SIZE];
    char warning_response[BUFF_SIZE], error_response[BUFF_SIZE], position_response[BUFF_SIZE];
    struct mess_node *nodeptr;
    int rtn_state;
    long motorData;
    char buff[BUFF_SIZE];
    struct LinMotController *cntrl;
    msta_field status;

    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;

    motor_info = &(motor_state[card]->motor_info[signal]);
    nodeptr = motor_info->motor_motion;
    status.All = motor_info->status.All;

    /* Request the status of this motor */
    sprintf(command, "!EW%c", signal+ASCII_0_TO_A);
    send_recv_mess(card, command, warning_response);
    Debug(2, "set_status, warning query, card %d, response=%s\n", card, warning_response);

    status.Bits.RA_PLUS_LS = 0;
    status.Bits.RA_MINUS_LS = 0;

    /* The response string is an integer representation of a bit sequence */
    
	int warning_code = atoi(warning_response);
	int moving_bit = 9;
	int moving = (warning_code & ( 1 << moving_bit )) >> moving_bit;
    status.Bits.RA_DONE = moving ? 0 : 1;
    Debug(2, "set_status, warning query, card %d, response=%d, moving=%d, done=%d\n", card, warning_code, moving, status.Bits.RA_DONE);
	
    /* Request the error state of the motor */
    sprintf(command, "!EE%c", signal+ASCII_0_TO_A);
    send_recv_mess(card, command, error_response);
	int error_code = atoi(error_response);
    Debug(2, "set_status, error query, card %d, response=%d\n", card, error_code);
    status.Bits.RA_PROBLEM = error_code > 0;

    /* encoder status */
    status.Bits.EA_SLIP       = 0;
    status.Bits.EA_POSITION   = 0;
    status.Bits.EA_SLIP_STALL = 0;
    status.Bits.EA_HOME       = 0;

    /* Request the position of this motor */
    sprintf(command, "!GP%c", signal+ASCII_0_TO_A);
    send_recv_mess(card, command, position_response);
    motorData = atoi(position_response);
    Debug(2, "set_status, position query, card %d, response=%s\n", card, position_response);
	
    if (motorData == motor_info->position)
    {
		if (nodeptr != 0)   /* Increment counter only if motor is moving. */
			motor_info->no_motion_count++;
    }
    else
    {
		status.Bits.RA_DIRECTION = (motorData >= motor_info->position) ? 1 : 0;
		motor_info->position = motorData;
		motor_info->encoder_position = motorData;
		motor_info->no_motion_count = 0;
    }

    rtn_state = (!motor_info->no_motion_count || 
        status.Bits.RA_DONE | status.Bits.RA_PROBLEM) ? 1 : 0;

    /* Test for post-move string. */
    if (status.Bits.RA_DONE && nodeptr != 0 &&
    nodeptr->postmsgptr != 0)
    {
        strcpy(buff, nodeptr->postmsgptr);
        strcat(buff, "\r");
        send_mess(card, buff, (char*) NULL);
        nodeptr->postmsgptr = NULL;
    }

    motor_info->status.All = status.All;
    Debug(2, "set_status, return value=%d\n", rtn_state);
    return (rtn_state);
}

/*********************************************************
 * Send a message to the LinMot board.   
 *********************************************************/
STATIC RTN_STATUS send_mess(int card, const char *com, const char *name)
{	
    char temp[BUFF_SIZE];
    Debug(2, "send_mess: sending message via send_recv_mess to card %d, message=%s\n", card, com);
	return (RTN_STATUS)send_recv_mess(card, com, temp);
}

/*********************************************************
 * Receive a message from the LinMot board.   
 *********************************************************/
STATIC int recv_mess(int card, char *com, int flag)
{
    double timeout;
    int flush;
    asynStatus status;
    size_t nread=0;
    int eomReason;
    struct LinMotController *cntrl;

    com[0] = '\0';
    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("resv_mess - invalid card #%d\n", card);
        return(-1);
    }

    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;

    if (flag == FLUSH) {
        flush = 1;
        timeout = 0;
    } else {
        flush = 0;
        timeout = TIMEOUT;
    }
    if (flush) status = pasynOctetSyncIO->flush(cntrl->pasynUser);
    status = pasynOctetSyncIO->read(cntrl->pasynUser, com, BUFF_SIZE,
                                    timeout, &nread, &eomReason);

    /* The response from the LinMot is terminated with CR/LF.  Remove these */
    if (nread == 0) com[0] = '\0';
    if (nread > 0) {
		/* Get rid of the preliminary # */
		memmove (com, com+1, strlen (com+1) + 1);
        Debug(2, "recv_mess: card %d, flag=%d, message = \"%s\"\n", card, flag, com);
    }
    if (nread == 0) {
        if (flag != FLUSH)  {
            Debug(1, "recv_mess: card %d read ERROR: no response\n", card);
        } else {
            Debug(3, "recv_mess: card %d flush returned no characters\n", card);
        }
    }
    return(strlen(com));
}

/************************************************************
 * Send a message to the LinMot board and receive a resonse   
 ************************************************************/
STATIC int send_recv_mess(int card, const char *out, char *response)
{
    char *p, *tok_save;
    struct LinMotController *cntrl;
    asynStatus status;
    size_t nwrite=0, nread=0;
    int eomReason;
    char temp[BUFF_SIZE];

    response[0] = '\0';

    /* Check that card exists */
    if (!motor_state[card])
    {
        epicsPrintf("send_recv_mess - invalid card #%d\n", card);
        return (-1);
    }

    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;

    /* Device support can send us multiple commands separated with ';'
     * characters.  The LinMot cannot handle more than 1 command on a line
     * so send them separately */
    strcpy(temp, out);
    for (p = epicsStrtok_r(temp, ";", &tok_save);
                ((p != NULL) && (strlen(p) != 0));
                p = epicsStrtok_r(NULL, ";", &tok_save)) {
        Debug(2, "send_recv_mess: sending message to card %d, message=%s\n", card, p);
		status = pasynOctetSyncIO->writeRead(cntrl->pasynUser, p, strlen(p),
                         response, BUFF_SIZE, TIMEOUT,
                         &nwrite, &nread, &eomReason);
    }

    /* The response from the LinMot is terminated with CR/LF.  Remove these */
    if (nread == 0) response[0] = '\0';
    if (nread > 0) {
		/* Get rid of the preliminary # */
		memmove(response, response+1, strlen (response+1) + 1);
        Debug(2, "send_recv_mess: card %d, response=%s\n", card, response);
    }
    if (nread == 0) {
        Debug(1, "send_recv_mess: card %d ERROR: no response\n", card);
    }
    return(strlen(response));
}

/************************************************************
 * Set up the LinMot motor
 * Scan rat is in units of 1/60 seconds
 ************************************************************/
RTN_STATUS LinMotSetup(int num_cards, int scan_rate)
{
    int itera;

    if (num_cards < 1 || num_cards > LinMot_NUM_CARDS)
        LinMot_num_cards = LinMot_NUM_CARDS;
    else
        LinMot_num_cards = num_cards;

    /* Set motor polling task rate */
    if (scan_rate >= 1 && scan_rate <= 60)
    targs.motor_scan_rate = scan_rate;
    else
    targs.motor_scan_rate = SCAN_RATE;

   /*
    * Allocate space for motor_state structure pointers.  Note this must be done
    * before LinMotConfig is called, so it cannot be done in motor_init()
    * This means that we must allocate space for a card without knowing
    * if it really exists, which is not a serious problem since this is just
    * an array of pointers.
    */
    motor_state = (struct controller **) malloc(LinMot_num_cards *
                                                sizeof(struct controller *));

    for (itera = 0; itera < LinMot_num_cards; itera++)
        motor_state[itera] = (struct controller *) NULL;
    return(OK);
}

RTN_STATUS LinMotConfig(int card,            /* Card being configured */
                        const char *port,    /* Asyn port name */
                        int n_axes)          /* Number of axes */
{
    struct LinMotController *cntrl;

    if (card < 0 || card >= LinMot_num_cards)
        return (ERROR);

    if (n_axes == 0) n_axes=1;  /* This is a new parameter, some startup files don't have it yet */
    motor_state[card] = (struct controller *) malloc(sizeof(struct controller));
    motor_state[card]->DevicePrivate = malloc(sizeof(struct LinMotController));
    cntrl = (struct LinMotController *) motor_state[card]->DevicePrivate;
    cntrl->n_axes = n_axes;
	/* Will be updated later but this is the default for a LinMot */
	cntrl->speed_resolution = 190735;
    strcpy(cntrl->port, port);
    return(OK);
}

/************************************************************
 * Initialise motor software/hardware
 ************************************************************/
STATIC int motor_init()
{
    struct controller *brdptr;
    struct LinMotController *cntrl;
    int card_index, motor_index;
    char response[BUFF_SIZE], command[BUFF_SIZE];
    int total_axis = 0;
    int status;
    bool success_rtn;

    initialized = true;   /* Indicate that driver is initialized. */

    /* Check for setup */
    if (LinMot_num_cards <= 0)
    {
        Debug(1, "motor_init: *LinMot driver disabled*\n");
        Debug(1, "LinMotSetup() is missing from startup script.\n");
        return (ERROR);
    }

    for (card_index = 0; card_index < LinMot_num_cards; card_index++)
    {
        if (!motor_state[card_index])
            continue;

        brdptr = motor_state[card_index];
        total_cards = card_index + 1;
        cntrl = (struct LinMotController *) brdptr->DevicePrivate;

        /* Initialize communications channel */
        success_rtn = false;

        status = pasynOctetSyncIO->connect(cntrl->port, 0, &cntrl->pasynUser, NULL);
        success_rtn = (status == asynSuccess);
        Debug(1, "motor_init, return from pasynOctetSyncIO->connect for port %s = %d, pasynUser=%p\n", cntrl->port, success_rtn, cntrl->pasynUser);

        if (success_rtn == true)
        {
            int retry = 0;

            /* Send a message to the board, see if it exists */
            /* flush any junk at input port - should not be any data available */
            do {
                recv_mess(card_index, response, FLUSH);
            } while (strlen(response) != 0);

            do
            {
				sprintf(command, "!VI%c;", card_index + ASCII_0_TO_A);
                send_recv_mess(card_index, command, response);
                retry++;
                /* Return value is length of response string */
            } while(strlen(response) == 0 && retry < 3);
        }

        if (success_rtn == true && strlen(response) > 0)
        {
            brdptr->localaddr = (char *) NULL;
            brdptr->motor_in_motion = 0;
            /* Leave bdptr->cmnd_response false because we read each response */
            /* in send_mess and send_recv_mess. */
            brdptr->cmnd_response = false;

			cntrl->speed_resolution = atoi(response);
            total_axis = cntrl->n_axes;
            brdptr->total_axis = total_axis;
            start_status(card_index);
            for (motor_index = 0; motor_index < total_axis; motor_index++)
            {
                struct mess_info *motor_info = &brdptr->motor_info[motor_index];

                motor_info->motor_motion = NULL;
                motor_info->status.All = 0;
                motor_info->no_motion_count = 0;
                motor_info->encoder_position = 0;
                motor_info->position = 0;
                set_status(card_index, motor_index);  /* Read status of each motor */
            }
        }
        else {
            motor_state[card_index] = (struct controller *) NULL;
		}
    }

    any_motor_in_motion = 0;

    Debug(3, "motor_init: spawning motor task\n");

    mess_queue.head = (struct mess_node *) NULL;
    mess_queue.tail = (struct mess_node *) NULL;

    free_list.head = (struct mess_node *) NULL;
    free_list.tail = (struct mess_node *) NULL;

    epicsThreadCreate((char *) "tLinMot", epicsThreadPriorityMedium,
              epicsThreadGetStackSize(epicsThreadStackMedium),
              (EPICSTHREADFUNC) motor_task, (void *) &targs);

    return(OK);
}
