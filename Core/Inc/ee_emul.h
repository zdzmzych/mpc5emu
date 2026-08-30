/*
 * ee_emul.h
 *
 *  Created on: 26 sie 2026
 *      Author: mzych
 */

#ifndef INC_EE_EMUL_H_
#define INC_EE_EMUL_H_

void EE_Emul_Init(void);

void EE_Emul_CS_Activate(void);
void EE_Emul_CS_Deactivate(void);

uint8_t EE_Emul_SPI_RxTx(uint8_t rx);

void EE_Emul_Process(void);

uint8_t EE_Emul_Read(uint16_t address);
void EE_Emul_Write(uint16_t address, uint8_t value);

#endif /* INC_EE_EMUL_H_ */
