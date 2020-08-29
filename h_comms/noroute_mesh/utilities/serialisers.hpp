#ifndef SERIALISERS_H
#define SERIALISERS_H

#include <stdint.h>

/**
 * @brief This namespace contains the functionalities to serialize object and values into bytes array.
 * 
 */
namespace serialisers
{
    /**
     * @brief Copies a uint8_t value into a uint8_t bytes array.
     * 
     * @param dest uint8_t array to store the copied bytes. MUST be at least 1 byte length.
     * @param src uint8_t value to copy.
     */
    void copyU8(uint8_t* dest, uint8_t src);

    /**
     * @brief Copies a uint16_t value into a uint8_t bytes array.
     * 
     * @param dest uint8_t array to store the copied bytes. MUST be at least 2 byte length.
     * @param src uint16_t value to copy.
     */
    void copyU16(uint8_t* dest, uint16_t src);

    /**
     * @brief Copies a uint32_t value into a uint8_t bytes array.
     * 
     * @param dest uint8_t array to store the copied bytes. MUST be at least 4 bytes length.
     * @param src uint32_t value to copy.
     */
    void copyU32(uint8_t* dest, uint32_t src);

    /**
     * @brief Retrieves a uint8_t value from a uint8_t bytes array.
     * 
     * @param src uint8_t array to retrieve from. MUST be at least 1 byte length.
     * @return uint8_t value retrieved.
     */
    uint8_t getU8(uint8_t* src);

    /**
     * @brief Retrieves a uint16_t value from a uint8_t bytes array.
     * 
     * @param src uint8_t array to retrieve from. MUST be at least 1 byte length.
     * @return uint16_t value retrieved.
     */
    uint16_t getU16(uint8_t* src);

    /**
     * @brief Retrieves a uint32_t value from a uint8_t bytes array.
     * 
     * @param src uint8_t array to retrieve from. MUST be at least 4 bytes length.
     * @return uint32_t value retrieved.
     */
    uint32_t getU32(uint8_t* src);
}

#endif // SERIALISERS_H
