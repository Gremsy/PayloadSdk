"""
Gremsy Payload SDK - Parameter Definitions
This file automatically loads the correct definitions based on PAYLOAD_TYPE from config.py

Supported payload types:
- VIO: Full-featured payload with tracking, EIS, noise reduction, etc.
- ORUSL: Payload with defog fan control (PAYLOAD_FAN_DEFOG) and advanced features
- ZIO: Simplified payload with basic EO camera controls
- MB1: Mini payload with basic zoom and IR capabilities

Usage:
    from payload_define import *

    # The correct definitions will be automatically imported based on PAYLOAD_TYPE in config.py
    # For ORUSL: PAYLOAD_FAN_DEFOG will be available
    # For VIO: Full set of tracking and EIS features
    # For ZIO: Basic camera controls
    # For MB1: Basic zoom and storage selection
"""

import sys
import os

# Import configuration to get PAYLOAD_TYPE
try:
    from config import PAYLOAD_TYPE
except ImportError:
    print("[ERROR] Failed to import PAYLOAD_TYPE from config.py")
    print("[ERROR] Make sure config.py exists and defines PAYLOAD_TYPE")
    sys.exit(1)

# Validate PAYLOAD_TYPE
SUPPORTED_PAYLOAD_TYPES = ["VIO", "ORUSL", "ZIO", "MB1"]
if PAYLOAD_TYPE not in SUPPORTED_PAYLOAD_TYPES:
    print(f"[ERROR] Invalid PAYLOAD_TYPE: {PAYLOAD_TYPE}")
    print(f"[ERROR] Supported types: {', '.join(SUPPORTED_PAYLOAD_TYPES)}")
    sys.exit(1)

# Import the correct definitions based on PAYLOAD_TYPE
print(f"[INFO] Loading payload definitions for {PAYLOAD_TYPE}")

if PAYLOAD_TYPE == "VIO":
    from vio_define import *
    import vio_define
    _define_module = vio_define
    print("[INFO] VIO definitions loaded successfully")

elif PAYLOAD_TYPE == "ORUSL":
    from orusl_define import *
    import orusl_define
    _define_module = orusl_define
    print("[INFO] ORUSL definitions loaded successfully (includes PAYLOAD_FAN_DEFOG)")

elif PAYLOAD_TYPE == "ZIO":
    from zio_define import *
    import zio_define
    _define_module = zio_define
    print("[INFO] ZIO definitions loaded successfully")

elif PAYLOAD_TYPE == "MB1":
    from mb1_define import *
    import mb1_define
    _define_module = mb1_define
    print("[INFO] MB1 definitions loaded successfully")

# Export PAYLOAD_TYPE and all symbols from the imported define module
# Note: When using 'from module import *', we need to explicitly define __all__
# to re-export all symbols from the imported define files
__all__ = ['PAYLOAD_TYPE']
__all__.extend([name for name in dir(_define_module) if not name.startswith('_')])
