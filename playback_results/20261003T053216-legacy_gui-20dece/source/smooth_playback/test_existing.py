"""Run unchanged repository tests from a real file, allowing spawned simulators.

The audit hook is installed in both parent and multiprocessing children. It
prohibits real CAN socket creation; this is not a physical test.
"""
import socket
import sys
import unittest


def audit(event,args):
    if event=='socket.__new__' and len(args)>1 and args[1]==socket.PF_CAN:
        raise RuntimeError('Offline regression suite forbids physical CAN')


sys.addaudithook(audit)

if __name__=='__main__':
    suite=unittest.defaultTestLoader.discover('tests',pattern='test_*.py')
    result=unittest.TextTestRunner(verbosity=1).run(suite)
    raise SystemExit(not result.wasSuccessful())
