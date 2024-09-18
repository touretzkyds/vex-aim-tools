from enum import Enum
from typing import Union
import time
class vexEnum:
    '''Base class for all enumerated types'''
    value = 0
    name = ""

    def __init__(self, value, name):
        self.value = value
        self.name = name

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

    def __hash__(self):
        return self.value

class SoundType(str, Enum):
   SIREN          = "siren"    
   WRONG_WAY      = "wrongWay"
   WRONG_WAY_SLOW = "wrongWaySlow"
   FILLUP         = "fillup"
   HEADLIGHTS_ON  = "headlightsOn"
   HEADLIGHTS_OFF = "headlightsOff"
   TOLLBOOTH      = "tollBooth"
   ALARM          = "alarm1"   
   TADA           = "tada"     
   DOOR_CLOSE     = "doorClose"
   RATCHET        = "ratchet"  
   WRENCH         = "wrench"   
   SIREN2         = "siren2"   
   RATCHET2       = "ratchet2" 
   ALARM2         = "alarm2"   
   POWER_DOWN     = "powerDown"

class KickType(str, Enum):
   SOFT          = "kick_soft"
   MEDIUM        = "kick_medium"
   HARD          = "kick_hard"

class AxisType(Enum):
   XAXIS        = 0
   YAXIS        = 1
   ZAXIS        = 2

class TurnType(Enum):
   LEFT        = 0
   RIGHT       = 1

vexnumber = Union[int, float]

class LightType(str, Enum):
   LIGHT1      = "light1"
   LIGHT2      = "light2"
   LIGHT3      = "light3"
   LIGHT4      = "light4"
   LIGHT5      = "light5"
   LIGHT6      = "light6"
   ALL         = "all"

class Color:
    '''### Color class - create a new color

    This class is used to create instances of color objects

    #### Arguments:
        value : The color value, can be specified in various ways, see examples.

    #### Returns:
        An instance of the Color class

    #### Examples:
        # create blue using hex value\\
        c = Color(0x0000ff)\n
        # create blue using r, g, b values\\
        c = Color(0, 0, 255)\n
        # create blue using web string\\
        c = Color("#00F")\n
        # create blue using web string (alternate)\\
        c = Color("#0000FF")\n
        # create red using an existing object\\
        c = Color(Color.RED)
    '''
    class DefinedColor:
        def __init__(self, value):
            self.value = value

    BLACK       = DefinedColor(0x000000)
    '''predefined Color black'''
    WHITE       = DefinedColor(0xFFFFFF)
    '''predefined Color white'''
    RED         = DefinedColor(0xFF0000)
    '''predefined Color red'''
    GREEN       = DefinedColor(0x00FF00)
    '''predefined Color green'''
    BLUE        = DefinedColor(0x0000FF)
    '''predefined Color blue'''
    YELLOW      = DefinedColor(0xFFFF00)
    '''predefined Color yellow'''
    ORANGE      = DefinedColor(0xFF8500)
    '''predefined Color orange'''
    PURPLE      = DefinedColor(0xFF00FF)
    '''predefined Color purple'''
    CYAN        = DefinedColor(0x00FFFF)
    '''predefined Color cyan'''
    TRANSPARENT = DefinedColor(0x000000)
    '''predefined Color transparent'''

    def __init__(self, *args):
        if len(args) == 1 and isinstance(args[0], int):
            self.value: int = args[0]
        elif len(args) == 3 and all(isinstance(arg, int) for arg in args):
            self.value = ((args[0] & 0xFF) << 16) + ((args[1] & 0xFF) << 8) + (args[2] & 0xFF)
        else:
            raise TypeError("bad parameters")
    
    def set_rgb(self, *args):
        '''### change existing Color instance to new rgb value

        #### Arguments:
            value : The color value, can be specified in various ways, see examples.

        #### Returns:
            integer value representing the color

        #### Examples:
            # create a color that is red
            c = Color(0xFF0000)
            # change color to blue using single value
            c.rgb(0x0000FF)
            # change color to green using three values
            c.rgb(0, 255, 0)
        '''
        if len(args) == 1 and isinstance(args[0], int):
            self.value = args[0]
        if len(args) == 3 and all(isinstance(arg, int) for arg in args):
            self.value = ((args[0] & 0xFF) << 16) + ((args[1] & 0xFF) << 8) + (args[2] & 0xFF)
    
    # ----------------------------------------------------------
class TimeUnits(str, Enum):
    '''The measurement units for time values.'''

    SECONDS  = "SECONDS"
    SEC      = "SECONDS"
    MSEC     = "MSEC"

def sleep(duration: vexnumber, units=TimeUnits.MSEC):
    '''### delay the current thread for the provided number of seconds or milliseconds.

    #### Arguments:
        duration: The number of seconds or milliseconds to sleep for
        units:    The units of duration, optional, default is milliseconds

    #### Returns:
        None
    '''
    if units == TimeUnits.MSEC:
        time.sleep(duration / 1000)
    else:
        time.sleep(duration)

class Emoji:

    class EmojiType:
        def __init__(self, name, value):
            self.name = name
            self.value = value

    SMILE           = EmojiType("smile",         0)
    SAD             = EmojiType("sad",           1)
    SILLY           = EmojiType("silly",         2)
    SURPRISED       = EmojiType("surprised",     3)
    THINKING        = EmojiType("thinking",      4)
    FEAR            = EmojiType("fear",          5)
    ANGRY           = EmojiType("angry",         6)
    LOVED           = EmojiType("loved",         7)
    DISGUST         = EmojiType("disgust",       8)
    ANNOYED         = EmojiType("annoyed",       9)
    PROUD           = EmojiType("proud",        10)
    LAUGHING        = EmojiType("laughing",     11)
    QUIET           = EmojiType("quiet",        12)
    EXCITED         = EmojiType("excited",      13)
    BORED           = EmojiType("bored",        14)
    CONFUSED        = EmojiType("confused",     15)
    CONFIDENT       = EmojiType("confident",    16)
    CHEERFUL        = EmojiType("cheerful",     17)
    EMBARRASSED     = EmojiType("embarrassed",  18)
    FRUSTRATED      = EmojiType("frustrated",   19)
    SHOCKED         = EmojiType("shocked",      20)
    JEALOUS         = EmojiType("jealous",      21)
    TIRED           = EmojiType("tired",        22)
    SICK            = EmojiType("sick",         23)
    NERVOUS         = EmojiType("nervous",      24)
    STRESSED        = EmojiType("stressed",     25)
    WORRIED         = EmojiType("worried",      26)
    SHY             = EmojiType("shy",          27)
    DISAPPOINTED    = EmojiType("disappointed", 28)
    THRILLED        = EmojiType("thrilled",     29)
    RELAXED         = EmojiType("relaxed",      30)

class EmojiLook:
    class EmojiLookType:
        def __init__(self, name, value):
            self.name = name
            self.value = value

    CENTER          = EmojiLookType("center", 0)
    LEFT            = EmojiLookType("left",   1)
    RIGHT           = EmojiLookType("right",  2)
