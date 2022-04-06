#ifndef HUSKYLENSUTILS_H
#define HUSKYLENSUTILS_H

#include "HUSKYLENS.h"

/// tag detection ///
HUSKYLENSResult getResult(HUSKYLENS *huskylens);
bool isResultValid(HUSKYLENSResult result);
int getResultID(HUSKYLENSResult result);

/// orientation ///
extern int16_t xCenter;
bool get_side(HUSKYLENSResult result);
bool is_close(HUSKYLENSResult result, int16_t target_height, int16_t offset);
bool is_in_range(HUSKYLENSResult result, int16_t target_height, int16_t offset);
bool is_centered(HUSKYLENSResult result, int16_t offset);

#endif