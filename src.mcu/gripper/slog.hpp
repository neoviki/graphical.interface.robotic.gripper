#ifndef SLOG_H
#define SLOG_H
#include "Arduino.h"

#define _msg_status			String("[ STATUS       ] ")
#define _msg_info			  String("[ INFO         ] ")
#define _msg_success		String("[ SUCCESS      ] ")
#define _msg_nsuccess		String("[ UNSUCCESSFUL ] ")
#define _msg_empty      String("")
#define _msg_essential  String("[ STATUS       ] ")

#define msg_success		0
#define msg_nsuccess	1
#define	msg_status		2
#define msg_info	  	4
#define msg_empty     5
#define msg_essential 6
extern  boolean debug;
void slog(int _status, String msg);

#endif
