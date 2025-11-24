/*
 * cobs.h
 *
 *  Created on: Oct 3, 2024
 *      Author: rob
 */

#ifndef INC_COBS_H_
#define INC_COBS_H_

size_t cobsEncode(const void *data, size_t length, uint8_t *buffer);
size_t cobsDecode(const uint8_t *buffer, size_t length, void *data);

#endif /* INC_COBS_H_ */
