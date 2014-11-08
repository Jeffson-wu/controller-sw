/**
  ******************************************************************************
  * @file    gdi.h 
  * @author  Jari Rene Jensen
  * @version V1.0.0
  * @date    30-Sep -2014
  * @brief   Header for gdi
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 Xtel </center></h2>
  *
  ******************************************************************************
  */ 

 void gdi_init();
 void gdi_send_msg_on_monitor(char * response);
 void gdi_send_msg_response(char * response);
 int send_led_cmd(u16 fn, long TubeId);


