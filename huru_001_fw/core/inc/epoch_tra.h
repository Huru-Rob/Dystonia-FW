#ifndef EPOCH_TRA_H_INCLUDED
#define EPOCH_TRA_H_INCLUDED

typedef struct 
{
    epoch_transmission_state_e state;
    bool send_next;
    bool send_previous;
    bool send_complete;
    bool tail_pending_deletion;  
} epoch_transmission_status_t;

int32_t epoch_transmission_init(void);
epoch_transmission_state_e epoch_transmission_get_state(void);
int32_t epoch_transmit_next(void);
int32_t epoch_re_transmit(void);
int32_t epoch_transmit_complete(void);
int32_t epoch_transmission_update(void);

#endif