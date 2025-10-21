/*
 * ui_menu.h
 *
 *  Created on: Oct 16, 2025
 *      Author: Willy Ulises
 */

#ifndef INC_UI_MENU_H_
#define INC_UI_MENU_H_

#pragma once
#include "main.h"
#include "uart_gamepad.h"
#include <stdbool.h>

void Menu_Init(void);                      // pinta fondo y heladitos
void Menu_Tick(void);                      // parpadeo + entrada P1/P2
int  Menu_GetIndex(void);
bool Menu_ActionPressed(void);             // A (edge) de cualquiera


#endif /* INC_UI_MENU_H_ */
