/*
 * This file is part of the KAMBot project.
 * 
 *  Copyright (C) 2016 Frantisek Burian <bufran _at_ seznam.cz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef I2C_H
#define I2C_H

#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>
#include <alloca.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * NOMENCLATURE:
 * 
 * i2c_(read|write}_(be|le)(type)[_array]
 * 
 * (read|write) ... operation with i2c
 * (be|le) ........ endianness of the busoperation
 * (type) ......... the type of argument in operation. any of this list:
 *     uint8, int8, uint16, int16, uint32, int32, uint64, int64
 * [_array] ....... optional suffix, specifies that the operation works with
 *                  list of data, instead of single value. It is designed to
 *                  access many registers starting at first index in one 
 *                  bus transaction
 * 
 * EXAMPLE:
 * 
 * i2c_write_beuint16()       
 *    ... write a single unsigned 16-bit value to register with big endian byte
 *        ordering
 * 
 * i2c_write_leint16_array()  
 *    ... write an array of signed 16-bit values to specified registers, with
 *        little endian byte ordering
 */

// forward declarations

/**
 * Initialize I2C bus
 * @param busno Bus number (for Rpiv2 set to 1)
 * @return file descriptor on success
 * @return -1 on failure (errno is set)
 */
static inline int i2c_init(int busno);

/**
 * Close the I2C bus
 * @param fd file descriptor to close
 * @return 0 on success
 * @return -1 on failure (errno is set)
 */
static inline int i2c_close(int fd);

/**
 * Make one transaction on the I2C bus.
 * 
 * This function writes nw bytes from the bus, following nr bytes reading from the bus.
 * 
 * @param fd file descriptor
 * @param chip_addr I2C address of the chip
 * @param buffer data storage for the transaction
 * @param nw number of bytes to write
 * @param nr number of bytes to read
 * @return number bytes readed when success
 * @return -1 on error (errno is set)
 */
static inline int i2c_transact(int fd, int chip_addr, uint8_t *buffer, int nw, int nr);

// standard writes
static inline int i2c_write_beint08(int fd, int chip_addr, int reg, int8_t value);
static inline int i2c_write_beint16(int fd, int chip_addr, int reg, int16_t value);
static inline int i2c_write_beint32(int fd, int chip_addr, int reg, int32_t value);
static inline int i2c_write_beint64(int fd, int chip_addr, int reg, int64_t value);
static inline int i2c_write_beuint08(int fd, int chip_addr, int reg, uint8_t value);
static inline int i2c_write_beuint16(int fd, int chip_addr, int reg, uint16_t value);
static inline int i2c_write_beuint32(int fd, int chip_addr, int reg, uint32_t value);
static inline int i2c_write_beuint64(int fd, int chip_addr, int reg, uint64_t value);
static inline int i2c_write_leint08(int fd, int chip_addr, int reg, int8_t value);
static inline int i2c_write_leint16(int fd, int chip_addr, int reg, int16_t value);
static inline int i2c_write_leint32(int fd, int chip_addr, int reg, int32_t value);
static inline int i2c_write_leint64(int fd, int chip_addr, int reg, int64_t value);
static inline int i2c_write_leuint08(int fd, int chip_addr, int reg, uint8_t value);
static inline int i2c_write_leuint16(int fd, int chip_addr, int reg, uint16_t value);
static inline int i2c_write_leuint32(int fd, int chip_addr, int reg, uint32_t value);
static inline int i2c_write_leuint64(int fd, int chip_addr, int reg, uint64_t value);

// array writes
static inline int i2c_write_beint08_array(int fd, int chip_addr, int reg, int8_t *values, size_t count);
static inline int i2c_write_beint16_array(int fd, int chip_addr, int reg, int16_t *values, size_t count);
static inline int i2c_write_beint32_array(int fd, int chip_addr, int reg, int32_t *values, size_t count);
static inline int i2c_write_beint64_array(int fd, int chip_addr, int reg, int64_t *values, size_t count);
static inline int i2c_write_beuint08_array(int fd, int chip_addr, int reg, uint8_t *values, size_t count);
static inline int i2c_write_beuint16_array(int fd, int chip_addr, int reg, uint16_t *values, size_t count);
static inline int i2c_write_beuint32_array(int fd, int chip_addr, int reg, uint32_t *values, size_t count);
static inline int i2c_write_beuint64_array(int fd, int chip_addr, int reg, uint64_t *values, size_t count);
static inline int i2c_write_leint08_array(int fd, int chip_addr, int reg, int8_t *values, size_t count);
static inline int i2c_write_leint16_array(int fd, int chip_addr, int reg, int16_t *values, size_t count);
static inline int i2c_write_leint32_array(int fd, int chip_addr, int reg, int32_t *values, size_t count);
static inline int i2c_write_leint64_array(int fd, int chip_addr, int reg, int64_t *values, size_t count);
static inline int i2c_write_leuint08_array(int fd, int chip_addr, int reg, uint8_t *values, size_t count);
static inline int i2c_write_leuint16_array(int fd, int chip_addr, int reg, uint16_t *values, size_t count);
static inline int i2c_write_leuint32_array(int fd, int chip_addr, int reg, uint32_t *values, size_t count);
static inline int i2c_write_leuint64_array(int fd, int chip_addr, int reg, uint64_t *values, size_t count);

// standard reads
static inline int i2c_read_beint08(int fd, int chip_addr, int reg, int8_t *value);
static inline int i2c_read_beint16(int fd, int chip_addr, int reg, int16_t *value);
static inline int i2c_read_beint32(int fd, int chip_addr, int reg, int32_t *value);
static inline int i2c_read_beint64(int fd, int chip_addr, int reg, int64_t *value);
static inline int i2c_read_beuint08(int fd, int chip_addr, int reg, uint8_t *value);
static inline int i2c_read_beuint16(int fd, int chip_addr, int reg, uint16_t *value);
static inline int i2c_read_beuint32(int fd, int chip_addr, int reg, uint32_t *value);
static inline int i2c_read_beuint64(int fd, int chip_addr, int reg, uint64_t *value);
static inline int i2c_read_leint08(int fd, int chip_addr, int reg, int8_t *value);
static inline int i2c_read_leint16(int fd, int chip_addr, int reg, int16_t *value);
static inline int i2c_read_leint32(int fd, int chip_addr, int reg, int32_t *value);
static inline int i2c_read_leint64(int fd, int chip_addr, int reg, int64_t *value);
static inline int i2c_read_leuint08(int fd, int chip_addr, int reg, uint8_t *value);
static inline int i2c_read_leuint16(int fd, int chip_addr, int reg, uint16_t *value);
static inline int i2c_read_leuint32(int fd, int chip_addr, int reg, uint32_t *value);
static inline int i2c_read_leuint64(int fd, int chip_addr, int reg, uint64_t *value);

// array reads 
static inline int i2c_read_beint08_array(int fd, int chip_addr, int reg, int8_t *values, size_t count);
static inline int i2c_read_beint16_array(int fd, int chip_addr, int reg, int16_t *values, size_t count);
static inline int i2c_read_beint32_array(int fd, int chip_addr, int reg, int32_t *values, size_t count);
static inline int i2c_read_beint64_array(int fd, int chip_addr, int reg, int64_t *values, size_t count);
static inline int i2c_read_beuint08_array(int fd, int chip_addr, int reg, uint8_t *values, size_t count);
static inline int i2c_read_beuint16_array(int fd, int chip_addr, int reg, uint16_t *values, size_t count);
static inline int i2c_read_beuint32_array(int fd, int chip_addr, int reg, uint32_t *values, size_t count);
static inline int i2c_read_beuint64_array(int fd, int chip_addr, int reg, uint64_t *values, size_t count);
static inline int i2c_read_leint08_array(int fd, int chip_addr, int reg, int8_t *values, size_t count);
static inline int i2c_read_leint16_array(int fd, int chip_addr, int reg, int16_t *values, size_t count);
static inline int i2c_read_leint32_array(int fd, int chip_addr, int reg, int32_t *values, size_t count);
static inline int i2c_read_leint64_array(int fd, int chip_addr, int reg, int64_t *values, size_t count);
static inline int i2c_read_leuint08_array(int fd, int chip_addr, int reg, uint8_t *values, size_t count);
static inline int i2c_read_leuint16_array(int fd, int chip_addr, int reg, uint16_t *values, size_t count);
static inline int i2c_read_leuint32_array(int fd, int chip_addr, int reg, uint32_t *values, size_t count);
static inline int i2c_read_leuint64_array(int fd, int chip_addr, int reg, uint64_t *values, size_t count);

/**********************************************************************************************************
 * IMPLEMENTATION
 * 
 * 
 **********************************************************************************************************/

// HELPER FUNCTIONS ---------------------------------------------------------------------------------------
static inline void __wbeu08(uint8_t *buffer, uint8_t value)
{
  buffer[0] = (uint8_t)(value >> 0) & 0xFF;
}

static inline void __wbeu16(uint8_t *buffer, uint16_t value)
{
  buffer[0] = (uint8_t)(value >> 8) & 0xFF;
  buffer[1] = (uint8_t)(value >> 0) & 0xFF;
}

static inline void __wbeu32(uint8_t *buffer, uint32_t value)
{
  buffer[0] = (uint8_t)(value >> 24) & 0xFF;
  buffer[1] = (uint8_t)(value >> 16) & 0xFF;
  buffer[2] = (uint8_t)(value >> 8) & 0xFF;
  buffer[3] = (uint8_t)(value >> 0) & 0xFF;
}

static inline void __wbeu64(uint8_t *buffer, uint64_t value)
{
  buffer[0] = (uint8_t)(value >> 56) & 0xFF;
  buffer[1] = (uint8_t)(value >> 48) & 0xFF;
  buffer[2] = (uint8_t)(value >> 40) & 0xFF;
  buffer[3] = (uint8_t)(value >> 32) & 0xFF;
  buffer[4] = (uint8_t)(value >> 24) & 0xFF;
  buffer[5] = (uint8_t)(value >> 16) & 0xFF;
  buffer[6] = (uint8_t)(value >> 8) & 0xFF;
  buffer[7] = (uint8_t)(value >> 0) & 0xFF;
}

static inline void __wleu08(uint8_t *buffer, uint8_t value)
{
  buffer[0] = (uint8_t)(value >> 0) & 0xFF;
}

static inline void __wleu16(uint8_t *buffer, uint16_t value)
{
  buffer[1] = (uint8_t)(value >> 8) & 0xFF;
  buffer[0] = (uint8_t)(value >> 0) & 0xFF;
}

static inline void __wleu32(uint8_t *buffer, uint32_t value)
{
  buffer[3] = (uint8_t)(value >> 24) & 0xFF;
  buffer[2] = (uint8_t)(value >> 16) & 0xFF;
  buffer[1] = (uint8_t)(value >> 8) & 0xFF;
  buffer[0] = (uint8_t)(value >> 0) & 0xFF;
}

static inline void __wleu64(uint8_t *buffer, uint64_t value)
{
  buffer[7] = (uint8_t)(value >> 56) & 0xFF;
  buffer[6] = (uint8_t)(value >> 48) & 0xFF;
  buffer[5] = (uint8_t)(value >> 40) & 0xFF;
  buffer[4] = (uint8_t)(value >> 32) & 0xFF;
  buffer[3] = (uint8_t)(value >> 24) & 0xFF;
  buffer[2] = (uint8_t)(value >> 16) & 0xFF;
  buffer[1] = (uint8_t)(value >> 8) & 0xFF;
  buffer[0] = (uint8_t)(value >> 0) & 0xFF;
}

static inline uint8_t __rbeu08(uint8_t *buffer)
{
  return buffer[0];
}

static inline uint16_t __rbeu16(uint8_t *buffer)
{
  return ((uint16_t)buffer[0] << 8) |
         ((uint16_t)buffer[1] << 0);
}

static inline uint32_t __rbeu32(uint8_t *buffer)
{
  return ((uint32_t)buffer[0] << 24) |
         ((uint32_t)buffer[1] << 16) |
         ((uint32_t)buffer[2] << 8) |
         ((uint32_t)buffer[3] << 0);
}

static inline uint64_t __rbeu64(uint8_t *buffer)
{
  return ((uint64_t)buffer[0] << 56) |
         ((uint64_t)buffer[1] << 48) |
         ((uint64_t)buffer[2] << 40) |
         ((uint64_t)buffer[3] << 32) |
         ((uint64_t)buffer[4] << 24) |
         ((uint64_t)buffer[5] << 16) |
         ((uint64_t)buffer[6] << 8) |
         ((uint64_t)buffer[7] << 0);
}

static inline uint8_t __rleu08(uint8_t *buffer)
{
  return buffer[0];
}

static inline uint16_t __rleu16(uint8_t *buffer)
{
  return ((uint16_t)buffer[1] << 8) |
         ((uint16_t)buffer[0] << 0);
}

static inline uint32_t __rleu32(uint8_t *buffer)
{
  return ((uint32_t)buffer[3] << 24) |
         ((uint32_t)buffer[2] << 16) |
         ((uint32_t)buffer[1] << 8) |
         ((uint32_t)buffer[0] << 0);
}

static inline uint64_t __rleu64(uint8_t *buffer)
{
  return ((uint64_t)buffer[7] << 56) |
         ((uint64_t)buffer[6] << 48) |
         ((uint64_t)buffer[5] << 40) |
         ((uint64_t)buffer[4] << 32) |
         ((uint64_t)buffer[3] << 24) |
         ((uint64_t)buffer[2] << 16) |
         ((uint64_t)buffer[1] << 8) |
         ((uint64_t)buffer[0] << 0);
}

// I2C FUNCTIONS ---------------------------------------------------------------------------------------


static inline int i2c_init(int busno)
{
  char name[256];
  sprintf(name, "/dev/i2c-%d", busno);
  return open(name, O_RDWR);
}

static inline int i2c_close(int fd)
{
  close(fd);
  return 0;
}

//#define LOG
static inline int i2c_transact(int fd, int chip_addr, uint8_t *buffer, int nw, int nr)  
{
  int err = ioctl(fd, I2C_SLAVE, chip_addr);

  if (err != 0)
    return err;
  
  if (nw > 0) {
    err = write(fd, buffer, nw);

#ifdef LOG
    printf("I2C: %2x WRITE[%d] ", chip_addr, nw);
    for (int i=0; i<nw;i++)
      printf("%2x ", buffer[i]);

    printf("(%d written)\n", err);
#endif    
  }
  
  if (err < 0)
    return err;
  
  if (nr > 0) {
    err = read(fd, buffer, nr);
    
#ifdef LOG
    printf("I2C: %2x READ[%d] ", chip_addr, nr);
    for (int i=0; i<nr;i++)
      printf("%2x ", buffer[i]);

    printf("(%d readed)\n", err);
#endif    
  }
  
  return err;
}


// write single token
#define __I2C_WS(typ, zapisovatel)                                \
  uint8_t data[1 + sizeof(typ)];                                  \
  data[0] = reg;                                                  \
  zapisovatel(&data[1], (typ)value);                              \
  return i2c_transact(fd, chip_addr, data, 1 + sizeof(typ), 0);

// write array of tokens
#define __I2C_WA(typ, zapisovatel)                                \
  int sz = sizeof(typ);                                           \
  uint8_t *data = (uint8_t*)alloca(count * sz + 1);               \
  data[0] = reg;                                                  \
  for (int i = 0; i < count; i++)                                 \
    zapisovatel(&data[1 + i * sz], (typ)values[i]);               \
  return i2c_transact(fd, chip_addr, data, count * sz + 1, 0);

// read single token
#define __I2C_RS(typ, citatel)                                    \
  uint8_t data[sizeof(typ)];                                      \
  data[0] = reg;                                                  \
  int err = i2c_transact(fd, chip_addr, data, 1, sizeof(typ));    \
  if (err == sizeof(typ))                                         \
    *value = (typ)citatel(data);                                  \
  return err;
  
// read array of tokens
#define __I2C_RA(typ, citatel)                                    \
  int sz = sizeof(typ);                                           \
  uint8_t *data = (uint8_t*)alloca(count * sz + 1);               \
  data[0] = reg;                                                  \
  int err = i2c_transact(fd, chip_addr, data, 1, count * sz);     \
  if (err == (count * sz)) {                                      \
    for (int i = 0; i < count; i++)                               \
      values[i] = (typ)citatel(&data[i * sz]);                    \
    return count;                                                 \
  }                                                               \
  return err;
  
static inline int i2c_write_beint08(int fd, int chip_addr, int reg, int8_t value)  
{
  __I2C_WS(uint8_t, __wbeu08)
}

static inline int i2c_write_beint16(int fd, int chip_addr, int reg, int16_t value)  
{
  __I2C_WS(uint16_t, __wbeu16)
}

static inline int i2c_write_beint32(int fd, int chip_addr, int reg, int32_t value)  
{
  __I2C_WS(uint32_t, __wbeu32)
}

static inline int i2c_write_beint64(int fd, int chip_addr, int reg, int64_t value)  
{
  __I2C_WS(uint64_t, __wbeu64)
}

static inline int i2c_write_beuint08(int fd, int chip_addr, int reg, uint8_t value)  
{
  __I2C_WS(uint8_t, __wbeu08)
}

static inline int i2c_write_beuint16(int fd, int chip_addr, int reg, uint16_t value)  
{
  __I2C_WS(uint16_t, __wbeu16)
}

static inline int i2c_write_beuint32(int fd, int chip_addr, int reg, uint32_t value)  
{
  __I2C_WS(uint32_t, __wbeu32)
}

static inline int i2c_write_beuint64(int fd, int chip_addr, int reg, uint64_t value)  
{
  __I2C_WS(uint64_t, __wbeu64)
}

static inline int i2c_write_beint08_array(int fd, int chip_addr, int reg, int8_t *values, size_t count)
{
  __I2C_WA(uint8_t, __wbeu08)
}

static inline int i2c_write_beint16_array(int fd, int chip_addr, int reg, int16_t *values, size_t count)
{
  __I2C_WA(uint16_t, __wbeu16)
}

static inline int i2c_write_beint32_array(int fd, int chip_addr, int reg, int32_t *values, size_t count)
{
  __I2C_WA(uint32_t, __wbeu32)
}

static inline int i2c_write_beint64_array(int fd, int chip_addr, int reg, int64_t *values, size_t count)
{
  __I2C_WA(uint64_t, __wbeu64)
}

static inline int i2c_write_beuint08_array(int fd, int chip_addr, int reg, uint8_t *values, size_t count)
{
  __I2C_WA(uint8_t, __wbeu08)
}

static inline int i2c_write_beuint16_array(int fd, int chip_addr, int reg, uint16_t *values, size_t count)
{
  __I2C_WA(uint16_t, __wbeu16)
}

static inline int i2c_write_beuint32_array(int fd, int chip_addr, int reg, uint32_t *values, size_t count)
{
  __I2C_WA(uint32_t, __wbeu32)
}

static inline int i2c_write_beuint64_array(int fd, int chip_addr, int reg, uint64_t *values, size_t count)
{
  __I2C_WA(uint64_t, __wbeu64)
}

static inline int i2c_write_leint08(int fd, int chip_addr, int reg, int8_t value)  
{
  __I2C_WS(uint8_t, __wleu08)
}

static inline int i2c_write_leint16(int fd, int chip_addr, int reg, int16_t value)  
{
  __I2C_WS(uint16_t, __wleu16)
}

static inline int i2c_write_leint32(int fd, int chip_addr, int reg, int32_t value)  
{
  __I2C_WS(uint32_t, __wleu32)
}

static inline int i2c_write_leint64(int fd, int chip_addr, int reg, int64_t value)  
{
  __I2C_WS(uint64_t, __wleu64)
}

static inline int i2c_write_leuint08(int fd, int chip_addr, int reg, uint8_t value)  
{
  __I2C_WS(uint8_t, __wleu08)
}

static inline int i2c_write_leuint16(int fd, int chip_addr, int reg, uint16_t value)  
{
  __I2C_WS(uint16_t, __wleu16)
}

static inline int i2c_write_leuint32(int fd, int chip_addr, int reg, uint32_t value)  
{
  __I2C_WS(uint32_t, __wleu32)
}

static inline int i2c_write_leuint64(int fd, int chip_addr, int reg, uint64_t value)  
{
  __I2C_WS(uint64_t, __wleu64)
}

static inline int i2c_write_leint08_array(int fd, int chip_addr, int reg, int8_t *values, size_t count)
{
  __I2C_WA(uint8_t, __wleu08)
}

static inline int i2c_write_leint16_array(int fd, int chip_addr, int reg, int16_t *values, size_t count)
{
  __I2C_WA(uint16_t, __wleu16)
}

static inline int i2c_write_leint32_array(int fd, int chip_addr, int reg, int32_t *values, size_t count)
{
  __I2C_WA(uint32_t, __wleu32)
}

static inline int i2c_write_leint64_array(int fd, int chip_addr, int reg, int64_t *values, size_t count)
{
  __I2C_WA(uint64_t, __wleu64)
}

static inline int i2c_write_leuint08_array(int fd, int chip_addr, int reg, uint8_t *values, size_t count)
{
  __I2C_WA(uint8_t, __wleu08)
}

static inline int i2c_write_leuint16_array(int fd, int chip_addr, int reg, uint16_t *values, size_t count)
{
  __I2C_WA(uint16_t, __wleu16)
}

static inline int i2c_write_leuint32_array(int fd, int chip_addr, int reg, uint32_t *values, size_t count)
{
  __I2C_WA(uint32_t, __wleu32)
}

static inline int i2c_write_leuint64_array(int fd, int chip_addr, int reg, uint64_t *values, size_t count)
{
  __I2C_WA(uint64_t, __wleu64)
}

static inline int i2c_read_beint08(int fd, int chip_addr, int reg, int8_t *value)  
{
  __I2C_RS(int8_t, __rbeu08)
}

static inline int i2c_read_beint16(int fd, int chip_addr, int reg, int16_t *value)  
{
  __I2C_RS(int16_t, __rbeu16)
}

static inline int i2c_read_beint32(int fd, int chip_addr, int reg, int32_t *value)  
{
  __I2C_RS(int32_t, __rbeu32)
}

static inline int i2c_read_beint64(int fd, int chip_addr, int reg, int64_t *value)  
{
  __I2C_RS(int64_t, __rbeu64)
}

static inline int i2c_read_beuint08(int fd, int chip_addr, int reg, uint8_t *value)  
{
  __I2C_RS(uint8_t, __rbeu08)
}

static inline int i2c_read_beuint16(int fd, int chip_addr, int reg, uint16_t *value)  
{
  __I2C_RS(uint16_t, __rbeu16)
}

static inline int i2c_read_beuint32(int fd, int chip_addr, int reg, uint32_t *value)  
{
  __I2C_RS(uint32_t, __rbeu32)
}

static inline int i2c_read_beuint64(int fd, int chip_addr, int reg, uint64_t *value)  
{
  __I2C_RS(uint64_t, __rbeu64)
}

static inline int i2c_read_leint08(int fd, int chip_addr, int reg, int8_t *value)  
{
  __I2C_RS(int8_t, __rleu08)
}

static inline int i2c_read_leint16(int fd, int chip_addr, int reg, int16_t *value)  
{
  __I2C_RS(int16_t, __rleu16)
}

static inline int i2c_read_leint32(int fd, int chip_addr, int reg, int32_t *value)  
{
  __I2C_RS(int32_t, __rleu32)
}

static inline int i2c_read_leint64(int fd, int chip_addr, int reg, int64_t *value)  
{
  __I2C_RS(int64_t, __rleu64)
}

static inline int i2c_read_leuint08(int fd, int chip_addr, int reg, uint8_t *value)  
{
  __I2C_RS(uint8_t, __rleu08)
}

static inline int i2c_read_leuint16(int fd, int chip_addr, int reg, uint16_t *value)  
{
  __I2C_RS(uint16_t, __rleu16)
}

static inline int i2c_read_leuint32(int fd, int chip_addr, int reg, uint32_t *value)  
{
  __I2C_RS(uint32_t, __rleu32)
}

static inline int i2c_read_leuint64(int fd, int chip_addr, int reg, uint64_t *value)  
{
  __I2C_RS(uint64_t, __rleu64)
}

// array reads 

static inline int i2c_read_beint08_array(int fd, int chip_addr, int reg, int8_t *values, size_t count)
{
  __I2C_RA(uint8_t, __rbeu08)
}

static inline int i2c_read_beint16_array(int fd, int chip_addr, int reg, int16_t *values, size_t count)
{
  __I2C_RA(uint16_t, __rbeu16)
}

static inline int i2c_read_beint32_array(int fd, int chip_addr, int reg, int32_t *values, size_t count)
{
  __I2C_RA(uint32_t, __rbeu32)
}

static inline int i2c_read_beint64_array(int fd, int chip_addr, int reg, int64_t *values, size_t count)
{
  __I2C_RA(uint64_t, __rbeu64)
}

static inline int i2c_read_beuint08_array(int fd, int chip_addr, int reg, uint8_t *values, size_t count)
{
  __I2C_RA(uint8_t, __rbeu08)
}

static inline int i2c_read_beuint16_array(int fd, int chip_addr, int reg, uint16_t *values, size_t count)
{
  __I2C_RA(uint16_t, __rbeu16)
}

static inline int i2c_read_beuint32_array(int fd, int chip_addr, int reg, uint32_t *values, size_t count)
{
  __I2C_RA(uint32_t, __rbeu32)
}

static inline int i2c_read_beuint64_array(int fd, int chip_addr, int reg, uint64_t *values, size_t count)
{
  __I2C_RA(uint64_t, __rbeu64)
}

static inline int i2c_read_leint08_array(int fd, int chip_addr, int reg, int8_t *values, size_t count)
{
  __I2C_RA(uint8_t, __rleu08)
}

static inline int i2c_read_leint16_array(int fd, int chip_addr, int reg, int16_t *values, size_t count)
{
  __I2C_RA(uint16_t, __rleu16)
}

static inline int i2c_read_leint32_array(int fd, int chip_addr, int reg, int32_t *values, size_t count)
{
  __I2C_RA(uint32_t, __rleu32)
}

static inline int i2c_read_leint64_array(int fd, int chip_addr, int reg, int64_t *values, size_t count)
{
  __I2C_RA(uint64_t, __rleu64)
}

static inline int i2c_read_leuint08_array(int fd, int chip_addr, int reg, uint8_t *values, size_t count)
{
  __I2C_RA(uint8_t, __rleu08)
}

static inline int i2c_read_leuint16_array(int fd, int chip_addr, int reg, uint16_t *values, size_t count)
{
  __I2C_RA(uint16_t, __rleu16)
}

static inline int i2c_read_leuint32_array(int fd, int chip_addr, int reg, uint32_t *values, size_t count)
{
  __I2C_RA(uint32_t, __rleu32)
}

static inline int i2c_read_leuint64_array(int fd, int chip_addr, int reg, uint64_t *values, size_t count)
{
  __I2C_RA(uint64_t, __rleu64)
}


static inline int i2c_rmw_leuint16(int fd, int chip_addr, int reg, uint16_t clearbits, uint16_t setbits, uint16_t togglebits)
{
  uint16_t value;
  int err = i2c_read_leuint16(fd, chip_addr, reg, &value);
  if (err < 0)
    return err;

  return i2c_write_leuint16(fd, chip_addr, reg, ((value & ~clearbits) | setbits) ^ togglebits);
}

#ifdef __cplusplus
}
#endif

#endif /* I2C_H */

