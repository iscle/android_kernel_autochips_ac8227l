#ifndef _MT_ARCH_TIMER_H_
#define _MT_ARCH_TIMER_H_

int localtimer_set_next_event(unsigned int evt);
unsigned int localtimer_get_counter(void);

#endif
