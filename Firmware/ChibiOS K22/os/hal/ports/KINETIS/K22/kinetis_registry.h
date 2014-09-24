/*
    ChibiOS/HAL - Copyright (C) 2014 Fabio Utzig

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    K22DX kinetis_registry.h
 * @brief   K22DX capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef _KINETIS_REGISTRY_H_
#define _KINETIS_REGISTRY_H_

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    K22 capabilities
 * @{
 */
/* EXT attributes.*/

#define KINETIS_PORTA_IRQ_VECTOR    Vector12C
#define KINETIS_PORTB_IRQ_VECTOR    Vector130
#define KINETIS_PORTC_IRQ_VECTOR    Vector134
#define KINETIS_PORTD_IRQ_VECTOR    Vector138
#define KINETIS_PORTE_IRQ_VECTOR    Vector13C



/** @} */

#endif /* _KINETIS_REGISTRY_H_ */

/** @} */
