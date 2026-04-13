/*************************************************************************
Title:    Timing Configuration for CKT-XING-ADV
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2026 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#ifndef _TIMINGS_H_
#define _TIMINGS_H_

// Timings

#define LOOP_UPDATE_TIME_MS       50
#define SWITCH_UPDATE_TIME_MS     25
#define SWITCH_LONG_PRESS_THRESHOLD_MS 2000UL

#define DECISECS_SIGNAL_MAIN_GATE_DELAY        20
#define DECISECS_SIGNAL_MAIN_GATE_DROP         30

#define DECISECS_SIGNAL_4Q_GATE_DELAY          20
#define DECISECS_SIGNAL_4Q_GATE_DROP           30
#define DECISECS_SIGNAL_4Q_GATE_RISE_DELAY     10

#define DECISECS_SIGNAL_GATE_RISE              20

#endif
