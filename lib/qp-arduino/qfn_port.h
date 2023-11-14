/**
* @file
* @brief QF-nano port AVR ATmega, QV-nano kernel, GNU-AVR toolset, Arduino
* @cond
******************************************************************************
* Last Updated for Version: 6.8.2
* Date of the Last Update:  2021-07-07
*
*                    Q u a n t u m  L e a P s
*                    ------------------------
*                    Modern Embedded Software
*
* Copyright (C) 2005-2021 Quantum Leaps, LLC. All rights reserved.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <www.gnu.org/licenses>.
*
* Contact information:
* <www.state-machine.com/licensing>
* <info@state-machine.com>
******************************************************************************
* @endcond
*/
#ifndef QFN_PORT_H
#define QFN_PORT_H

#if defined(IS_CROSS)
#include "qfn_port_avr.h"
#elif defined(IS_NATIVE)
#include "qfn_port_native.h"
#elif defined(IS_TEST)
#include "qfn_port_native.h"
#endif
#endif /* QFN_PORT_H */
