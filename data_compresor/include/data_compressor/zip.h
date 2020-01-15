#ifndef _DC_ZIP_H
#define _DC_ZIP_H

#include <vector>
#include <stdint.h>

std::vector<uint8_t> compressZip(std::vector<uint8_t> packet);
std::vector<uint8_t> uncompressZip(std::vector<uint8_t> packet);

#endif