#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>

#ifndef __BIT_OPS__
#define __BIT_OPS__
/* bit operations */
#define REG64(addr)                  (*(volatile uint64_t *)(uint32_t)(addr))
#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#ifndef BIT 
#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
#endif /* BIT */ 
#define BITS(start, end)             ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end)))) 
#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))
#endif // __BIT_OPS__

/** @brief Number of bytes in @p x mebibytes */
#define MB(x) (KB(x) << 10)
/** @brief Number of bytes in @p x gibibytes */
#define GB(x) (MB(x) << 10)


/** @brief Number of Hz in @p x kHz */
#define KHZ(x) ((x) * 1000)
/** @brief Number of Hz in @p x MHz */
#define MHZ(x) (KHZ(x) * 1000)

/**
 *  @brief Put a 16-bit integer as big-endian to arbitrary location.
 *
 *  Put a 16-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 16-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_be16(uint16_t val, uint8_t dst[2])
{
	dst[0] = val >> 8;
	dst[1] = val;
}

/**
 *  @brief Put a 24-bit integer as big-endian to arbitrary location.
 *
 *  Put a 24-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 24-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_be24(uint32_t val, uint8_t dst[3])
{
	dst[0] = val >> 16;
	sys_put_be16(val, &dst[1]);
}

/**
 *  @brief Put a 32-bit integer as big-endian to arbitrary location.
 *
 *  Put a 32-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 32-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_be32(uint32_t val, uint8_t dst[4])
{
	sys_put_be16(val >> 16, dst);
	sys_put_be16(val, &dst[2]);
}
/**
 *  @brief Put a 40-bit integer as big-endian to arbitrary location.
 *
 *  Put a 40-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 40-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_be40(uint64_t val, uint8_t dst[5])
{
	dst[0] = val >> 32;
	sys_put_be32(val, &dst[1]);
}

/**
 *  @brief Put a 48-bit integer as big-endian to arbitrary location.
 *
 *  Put a 48-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 48-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_be48(uint64_t val, uint8_t dst[6])
{
	sys_put_be16(val >> 32, dst);
	sys_put_be32(val, &dst[2]);
}

/**
 *  @brief Put a 64-bit integer as big-endian to arbitrary location.
 *
 *  Put a 64-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in big-endian format.
 *
 *  @param val 64-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_be64(uint64_t val, uint8_t dst[8])
{
	sys_put_be32(val >> 32, dst);
	sys_put_be32(val, &dst[4]);
}

/**
 *  @brief Put a 16-bit integer as little-endian to arbitrary location.
 *
 *  Put a 16-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 16-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_le16(uint16_t val, uint8_t dst[2])
{
	dst[0] = val;
	dst[1] = val >> 8;
}

/**
 *  @brief Put a 24-bit integer as little-endian to arbitrary location.
 *
 *  Put a 24-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 24-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_le24(uint32_t val, uint8_t dst[3])
{
	sys_put_le16(val, dst);
	dst[2] = val >> 16;
}

/**
 *  @brief Put a 32-bit integer as little-endian to arbitrary location.
 *
 *  Put a 32-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 32-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_le32(uint32_t val, uint8_t dst[4])
{
	sys_put_le16(val, dst);
	sys_put_le16(val >> 16, &dst[2]);
}

/**
 *  @brief Put a 40-bit integer as little-endian to arbitrary location.
 *
 *  Put a 40-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 40-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_le40(uint64_t val, uint8_t dst[5])
{
	sys_put_le32(val, dst);
	dst[4] = val >> 32;
}

/**
 *  @brief Put a 48-bit integer as little-endian to arbitrary location.
 *
 *  Put a 48-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 48-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_le48(uint64_t val, uint8_t dst[6])
{
	sys_put_le32(val, dst);
	sys_put_le16(val >> 32, &dst[4]);
}

/**
 *  @brief Put a 64-bit integer as little-endian to arbitrary location.
 *
 *  Put a 64-bit integer, originally in host endianness, to a
 *  potentially unaligned memory location in little-endian format.
 *
 *  @param val 64-bit integer in host endianness.
 *  @param dst Destination memory address to store the result.
 */
static inline void sys_put_le64(uint64_t val, uint8_t dst[8])
{
	sys_put_le32(val, dst);
	sys_put_le32(val >> 32, &dst[4]);
}

/**
 *  @brief Get a 16-bit integer stored in big-endian format.
 *
 *  Get a 16-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 16-bit integer to get.
 *
 *  @return 16-bit integer in host endianness.
 */
static inline uint16_t sys_get_be16(const uint8_t src[2])
{
	return ((uint16_t)src[0] << 8) | src[1];
}

/**
 *  @brief Get a 24-bit integer stored in big-endian format.
 *
 *  Get a 24-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 24-bit integer to get.
 *
 *  @return 24-bit integer in host endianness.
 */
static inline uint32_t sys_get_be24(const uint8_t src[3])
{
	return ((uint32_t)src[0] << 16) | sys_get_be16(&src[1]);
}

/**
 *  @brief Get a 32-bit integer stored in big-endian format.
 *
 *  Get a 32-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 32-bit integer to get.
 *
 *  @return 32-bit integer in host endianness.
 */
static inline uint32_t sys_get_be32(const uint8_t src[4])
{
	return ((uint32_t)sys_get_be16(&src[0]) << 16) | sys_get_be16(&src[2]);
}

/**
 *  @brief Get a 40-bit integer stored in big-endian format.
 *
 *  Get a 40-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 40-bit integer to get.
 *
 *  @return 40-bit integer in host endianness.
 */
static inline uint64_t sys_get_be40(const uint8_t src[5])
{
	return ((uint64_t)sys_get_be32(&src[0]) << 8) | src[4];
}

/**
 *  @brief Get a 48-bit integer stored in big-endian format.
 *
 *  Get a 48-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 48-bit integer to get.
 *
 *  @return 48-bit integer in host endianness.
 */
static inline uint64_t sys_get_be48(const uint8_t src[6])
{
	return ((uint64_t)sys_get_be32(&src[0]) << 16) | sys_get_be16(&src[4]);
}

/**
 *  @brief Get a 64-bit integer stored in big-endian format.
 *
 *  Get a 64-bit integer, stored in big-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the big-endian 64-bit integer to get.
 *
 *  @return 64-bit integer in host endianness.
 */
static inline uint64_t sys_get_be64(const uint8_t src[8])
{
	return ((uint64_t)sys_get_be32(&src[0]) << 32) | sys_get_be32(&src[4]);
}

/**
 *  @brief Get a 16-bit integer stored in little-endian format.
 *
 *  Get a 16-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 16-bit integer to get.
 *
 *  @return 16-bit integer in host endianness.
 */
static inline uint16_t sys_get_le16(const uint8_t src[2])
{
	return ((uint16_t)src[1] << 8) | src[0];
}

/**
 *  @brief Get a 24-bit integer stored in little-endian format.
 *
 *  Get a 24-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 24-bit integer to get.
 *
 *  @return 24-bit integer in host endianness.
 */
static inline uint32_t sys_get_le24(const uint8_t src[3])
{
	return ((uint32_t)src[2] << 16) | sys_get_le16(&src[0]);
}

/**
 *  @brief Get a 32-bit integer stored in little-endian format.
 *
 *  Get a 32-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 32-bit integer to get.
 *
 *  @return 32-bit integer in host endianness.
 */
static inline uint32_t sys_get_le32(const uint8_t src[4])
{
	return ((uint32_t)sys_get_le16(&src[2]) << 16) | sys_get_le16(&src[0]);
}

/**
 *  @brief Get a 40-bit integer stored in little-endian format.
 *
 *  Get a 40-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 40-bit integer to get.
 *
 *  @return 40-bit integer in host endianness.
 */
static inline uint64_t sys_get_le40(const uint8_t src[5])
{
	return ((uint64_t)sys_get_le32(&src[1]) << 8) | src[0];
}

/**
 *  @brief Get a 48-bit integer stored in little-endian format.
 *
 *  Get a 48-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 48-bit integer to get.
 *
 *  @return 48-bit integer in host endianness.
 */
static inline uint64_t sys_get_le48(const uint8_t src[6])
{
	return ((uint64_t)sys_get_le32(&src[2]) << 16) | sys_get_le16(&src[0]);
}

/**
 *  @brief Get a 64-bit integer stored in little-endian format.
 *
 *  Get a 64-bit integer, stored in little-endian format in a potentially
 *  unaligned memory location, and convert it to the host endianness.
 *
 *  @param src Location of the little-endian 64-bit integer to get.
 *
 *  @return 64-bit integer in host endianness.
 */
static inline uint64_t sys_get_le64(const uint8_t src[8])
{
	return ((uint64_t)sys_get_le32(&src[4]) << 32) | sys_get_le32(&src[0]);
}

#endif // __UTILS_H__