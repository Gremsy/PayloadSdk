from enum import IntEnum, Enum

class IntEnumBase(IntEnum):
    """Base class for enums based on integers."""
    def __int__(self):
        return int(self.value)

class FloatEnumBase(Enum):
    """Base class for enums based on floats."""
    def __float__(self):
        return float(self.value)