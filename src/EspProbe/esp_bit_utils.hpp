#ifndef ESP_BIT_UTILS_HPP
#define ESP_BIT_UTILS_HPP

#include <cstdint>
#include <cstring>

namespace esp_probe
{

inline void buf_set_u32(uint8_t* buffer, unsigned offset, unsigned width, uint32_t value)
{
	for (unsigned i = 0; i < width; i++)
	{
		if ((value >> i) & 1)
			buffer[(offset + i) / 8] |= static_cast<uint8_t>(1u << ((offset + i) % 8));
		else
			buffer[(offset + i) / 8] &= static_cast<uint8_t>(~(1u << ((offset + i) % 8)));
	}
}

inline uint32_t buf_get_u32(const uint8_t* buffer, unsigned offset, unsigned width)
{
	uint32_t value = 0;
	for (unsigned i = 0; i < width; i++)
	{
		if (buffer[(offset + i) / 8] & (1u << ((offset + i) % 8)))
			value |= 1u << i;
	}
	return value;
}

inline uint32_t set_field_u32(uint32_t reg, uint32_t mask, uint32_t value)
{
	const uint32_t shift = static_cast<uint32_t>(__builtin_ctz(mask));
	return (reg & ~mask) | ((value << shift) & mask);
}

inline uint32_t get_field_u32(uint32_t reg, uint32_t mask)
{
	return reg & mask;
}

}  // namespace esp_probe

#endif
