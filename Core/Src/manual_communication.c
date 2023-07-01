
/*
uint8_t cfg_fr[] = { 0x02 , 0x31 , 0x30 , 0x45 , 0x30 , 0x45 , 0x33 , 0x03 } ;
  uint8_t cfg_sa[100] ;
  uint8_t cfg_sa_expected[] = { 0x02 , 0x39 , 0x31 , 0x36 , 0x38 , 0x37 , 0x32 , 0x03 } ;

  uint8_t cfg_rr[] = { 0x02 , 0x31 , 0x35 , 0x36 , 0x34 , 0x41 , 0x33 , 0x03 } ;
  uint8_t cfg_ra[100] ;
  uint8_t cfg_ra_expected[] = { 0x02 , 0x39 , 0x35 , 0x30 , 0x33 , 0x30 , 0x31 , 0x30 , 0x32 , 0x30 , 0x31 , 0x30 , 0x30 , 0x00 , 0x35 , 0x30 , 0x30 , 0x30 , 0x31 , 0x32 , 0x36 , 0x32 , 0x39 , 0x03 } ;

  astro_reset_state = HAL_GPIO_ReadPin ( GPIOA , ASTRO_RESET_Pin ) ;
  astro_event_state = HAL_GPIO_ReadPin ( ASTRO_EVENT_EXTI12_GPIO_Port , ASTRO_EVENT_EXTI12_Pin ) ;
  HAL_Delay ( 1000 ) ;

  result = HAL_UART_Transmit ( HUART_ASTRO , cfg_fr , 8 , 1000 ) ;
  result = HAL_UART_Receive ( HUART_ASTRO , cfg_sa , 8 , 1000 ) ;
  if ( strncmp( cfg_sa , cfg_sa_expected , sizeof ( cfg_sa_expected ) ) == 0 )
  	send_debug_logs ( "cfg_sa ok." ) ;
  else
  	send_debug_logs ( "cfg_sa not ok." ) ;
  result = HAL_UART_Transmit ( HUART_ASTRO , cfg_fr , 8 , 1000 ) ;
  result = HAL_UART_Receive ( HUART_ASTRO , cfg_sa , 8 , 1000 ) ;
  if ( strncmp( cfg_sa , cfg_sa_expected , sizeof ( cfg_sa_expected ) ) == 0 )
	send_debug_logs ( "cfg_sa ok." ) ;
  else
	send_debug_logs ( "cfg_sa not ok." ) ;
  result = HAL_UART_Transmit ( HUART_ASTRO , cfg_rr , 8 , 1000 ) ;
  result = HAL_UART_Receive ( HUART_ASTRO , cfg_ra , 24 , 1000 ) ;
  if ( strncmp( cfg_ra , cfg_ra_expected , sizeof ( cfg_ra_expected ) ) == 0 )
    send_debug_logs ( "cfg_ra ok." ) ;
  else
    send_debug_logs ( "cfg_ra not ok." ) ;
*/
