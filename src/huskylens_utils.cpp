#include "husklens_utils.h"

/// tag detection ///

void printResult(HUSKYLENSResult result)
{
    if (result.command == COMMAND_RETURN_BLOCK)
    {

        Serial.print(F("result.command : "));
        Serial.println(result.command);
        Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW)
    {
        Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
    }
    else
    {
        Serial.println("Object unknown!");
    }
}

// get huskylens result - Warning : result may be empty. Check command value
// note : huskylens ADDRESS must be passed to avoid memory issues
HUSKYLENSResult getResult(HUSKYLENS *huskylens)
{
    if (!(*huskylens).request())
    {
    } // Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    else if (!(*huskylens).isLearned())
    {
    } // Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if (!(*huskylens).available())
    {
    } // Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
        while ((*huskylens).available())
        {
            HUSKYLENSResult result = (*huskylens).read();
            return result;
        }
    }
    HUSKYLENSResult result;
    result.command = 0;
    return result;
}

// checks if result is a command return block
bool isResultValid(HUSKYLENSResult result)
{
    // Serial.print(F("result.command : "));
    // Serial.println(result.command);
    return (result.command == COMMAND_RETURN_BLOCK);
}

// get result id
int getResultID(HUSKYLENSResult result)
{
    return result.ID;
}

/// orientation ///
// returns which side on the display the target is currently on 0:left, 1:right
bool get_side(HUSKYLENSResult result)
{
    return (result.xCenter > xCenter);
}

bool is_close(HUSKYLENSResult result, int16_t target_height)
{
    return (result.height >= target_height);
}

bool is_in_range(HUSKYLENSResult result, int16_t target_height, int16_t offset)
{
    return (target_height - offset <= result.height && result.height <= target_height + offset);
}

bool is_centered(HUSKYLENSResult result, int16_t offset)
{
    return (xCenter - offset <= result.xCenter && result.xCenter <= xCenter + offset);
}

bool get_last_dir(HUSKYLENSResult result)
{
    return (result.xCenter <= xCenter);
}