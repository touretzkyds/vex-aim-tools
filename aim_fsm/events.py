"""
    The base Event class is imported from evbase.py.
    All other events are defined here.
"""

from .evbase import Event

class CompletionEvent(Event):
    """Signals completion of a state node's action."""
    pass


class SuccessEvent(Event):
    """Signals success of a state node's action."""
    def __init__(self,details=None):
        super().__init__()
        self.details = details


class FailureEvent(Event):
    """Signals failure of a state node's action."""
    def __init__(self,details=None):
        super().__init__()
        self.details = details

    def __repr__(self):
        return '<%s for %s: %s>' % (self.__class__.__name__, self.source.name, self.details)


class DataEvent(Event):
    """Signals a data item broadcasted by the node."""
    def __init__(self,data):
        super().__init__()
        self.data = data


class TextMsgEvent(Event):
    """Signals a text message broadcasted to the state machine."""
    def __init__(self,string,words=None,result=None):
        super().__init__()
        self.string = string
        self.words = words or string.split(None)
        self.result = result


class SpeechEvent(Event):
    """Results of speech recognition process."""
    def __init__(self,string,words=None,result=None):
        super().__init__()
        self.string = string
        self.words = words
        self.result = result

class PilotEvent(Event):
    """Results of a pilot request."""
    def __init__(self,status,**args):
        super().__init__()
        self.status = status
        self.args = args

    def __repr__(self):
        try:
            src_string = self.source.name
        except:
            src_string = repr(self.source)
        return '<%s %s from %s>' % (self.__class__.__name__, self.status.__name__, src_string)


#________________ Robot-generated events ________________

class TouchEvent(Event):
    def __init__(self, x, y, flags):
        super().__init__()
        self.x = x
        self.y = y
        self.flags = flags

    def __repr__(self):
        return f"<TouchEvent x:{self.x} y:{self.y} flags:{self.flags}>"

