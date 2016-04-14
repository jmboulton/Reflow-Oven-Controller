/*******************************************************************************
* Title: Reflow Oven Controller
* Version: 1.0
* Date: 15-04-2016
* Company: Veritide Ltd
* Author: James Boulton
* 
* ButtonType.h
* Used by ReadButtons() to return button state.
*/


typedef	enum BUTTON
{
  BUTTON_NONE,
  BUTTON_UP,
  BUTTON_DOWN,
  BUTTON_LEFT,
  BUTTON_RIGHT,
  BUTTON_SELECT
} button_t;

