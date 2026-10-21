"""Compatibility entry point for the replaced gesture UI test (no camera)."""
from check_gesture_ui import main
if __name__=='__main__':
    import sys
    main(*sys.argv[1:])
