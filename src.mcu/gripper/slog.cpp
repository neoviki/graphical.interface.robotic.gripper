#include "slog.hpp"
void slog(int _status, String msg)
{
	String message = "";

	if(debug == false){
		/* print only UNSUCCESSFUL EVENTS */
		if(_status != msg_nsuccess){
			return;
		}
	}

	switch(_status){
		case msg_success:
			message = _msg_success + msg;
			break;
		case msg_nsuccess:
			message = _msg_nsuccess + msg;
			break;
		case msg_status:
			message = _msg_status + msg;
			break;
		case msg_info:
			message = _msg_info + msg;
			break;
    case msg_empty:
      message = _msg_empty + msg;
      break;
    case msg_essential:
      message = _msg_status + msg;
      break;
		default:
			break;
	}
	Serial.print(message);
	return;
}
