from ctypes import cdll
lib = cdll.LoadLibrary('./libCamera.so')

class Foo(object):
    def __init__(self):
        self.obj = lib.newCamera()

    def bar(self):
        lib.streamCamera(self.obj)


c = Camera()
c.stream()