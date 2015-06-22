from ctypes import cdll
lib = cdll.LoadLibrary('./libCamera.so')

class Camera(object):
    def __init__(self):
        self.obj = lib.newCamera()

    def stream(self):
        lib.streamCamera(self.obj)


c = Camera()
c.stream()