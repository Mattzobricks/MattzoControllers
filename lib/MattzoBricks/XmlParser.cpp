#include <XmlParser.h>

// if not defined debug messages are off for this file
// #undef MC_DEBUG
// #undef MC_INFO
// #include <log4MC.h>

bool XmlParser::tryReadCharAttr(const char *xmlMessage, const char *attr, char **returnValue)
{
    char *pos = (char *)xmlMessage;
    char * value = nullptr;
    unsigned int attrLength = strlen(attr);
    bool foundStart = false;
    unsigned int start = 0, first, last;

    while ((pos = strstr(pos, (char *)attr)))
    { // YES, the '=' is correct
        // attribute can be at the start, or not and when not a ' ' should be in front it
        if (attr != pos)
        {
            // can do pos-1
            foundStart = (*(pos - 1) == ' ');
        }
        else
        {
            foundStart = true;
        }
        if ((foundStart) && (strlen(pos) > attrLength) && (*(pos + attrLength) == '='))
        {
            start = attrLength;
            first = 0;
            last = 0;
            while (start < strlen(pos))
            {
                if (!first && *(pos + start) == '"')
                {
                    first = start;
                }
                else if (first && !last && *(pos + start) == '"')
                {
                    // check if char before " is not a '\'
                    if (*(pos + start - 1) != '\\')
                    {
                        last = start;
                        break;
                    }
                }
                start++;
            }
            // found a first and last position of a double quoate
            if (last && first)
            {
                value = new char[last - first + 1];
                strncpy(value, pos + first + 1, last - first - 1);
                value[last - first - 1] = '\0';
                // pos+=last-1; // if you want to loop on
                break; // from the while loop, we expect to find only one attibute named 'attrName'
            }
        }
        pos++; // can do because we found cmd, start next search from "md"
    }

    // Serial.print("Found value for attr '");
    // Serial.print(attr);
    // Serial.print("': ");
    // Serial.println(value);
    
    *returnValue = value;

    return value != nullptr;
}

bool XmlParser::tryReadBoolAttr(const char *xmlMessage, const char *attr, bool *returnValue)
{
    bool success = false;
    char *attrValue;

    if (tryReadCharAttr(xmlMessage, attr, &attrValue))
    {
        success = tryParseBool(attrValue, returnValue);
    }

    delete[] attrValue;
    return success;
}

bool XmlParser::tryReadIntAttr(const char *xmlMessage, const char *attr, int *returnValue)
{
    bool success = false;
    char *attrValue;

    if (tryReadCharAttr(xmlMessage, attr, &attrValue))
    {
        success = tryParseInt(attrValue, returnValue);
    }

    delete[] attrValue;
    return success;
}

bool XmlParser::tryParseBool(const char *item, bool *returnValue)
{
    bool success = false;
    *returnValue = false;
    // MC_LOG_DEBUG("Scanning %s",item);

    if (strcmp(item, "false") == 0)
    {
        *returnValue = false;
        success = true;
    }
    else if (strcmp(item, "true") == 0)
    {
        *returnValue = true;
        success = true;
    }

    return success;
}

// for now a wrapper around atoi, later we can also make it more fancy
bool XmlParser::tryParseInt(const char *item, int *returnValue)
{
    bool success = false;
    *returnValue = 0;
    // MC_LOG_DEBUG("Scanning %s",item);

    if (isdigit(item[0]))
    {
        // assume a number
        *returnValue = atoi(item);
        success = true;
    }

    return success;
}