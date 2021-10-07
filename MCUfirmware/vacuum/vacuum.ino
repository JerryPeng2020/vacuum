/****************************************************
  /* 7Bot(V2) Arduino example
  /* Author: Jerry Peng
  /* Date: Sep. 21th, 2021
  /*
  /* Version 1.0 (equal to old version v2.3)
  /* www.7bot.cc
  /*
  /* Description: this example enable 7Bot receive command and
  /* send feedback through USB-UART port. 
  /*
  /*
  /***************************************************/



#include "Arm7Bot.h"

Arm7Bot arm;


void setup()
{
  arm.init();
}


void loop()
{
  arm.firmwareSystem();
}
