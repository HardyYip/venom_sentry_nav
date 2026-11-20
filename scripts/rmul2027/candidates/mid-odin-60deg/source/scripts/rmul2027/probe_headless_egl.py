"""Report the renderer of a real headless EGL OpenGL context."""

import ctypes as c
import json


def renderer():
    egl = c.CDLL("libEGL.so.1")
    pointer, integer, uint = c.c_void_p, c.c_int, c.c_uint

    def call(name, result_type, *arg_types):
        function = getattr(egl, name)
        function.restype = result_type
        function.argtypes = arg_types
        return function

    proc = call("eglGetProcAddress", pointer, c.c_char_p)
    query_devices = c.CFUNCTYPE(uint, integer, c.POINTER(pointer), c.POINTER(integer))(
        proc(b"eglQueryDevicesEXT"))
    get_display = c.CFUNCTYPE(pointer, uint, pointer, c.POINTER(integer))(
        proc(b"eglGetPlatformDisplayEXT"))
    initialize = call("eglInitialize", uint, pointer, c.POINTER(integer), c.POINTER(integer))
    terminate = call("eglTerminate", uint, pointer)
    bind_api = call("eglBindAPI", uint, uint)
    choose = call("eglChooseConfig", uint, pointer, c.POINTER(integer),
                  c.POINTER(pointer), integer, c.POINTER(integer))
    create_surface = call("eglCreatePbufferSurface", pointer, pointer, pointer, c.POINTER(integer))
    create_context = call("eglCreateContext", pointer, pointer, pointer, pointer, c.POINTER(integer))
    make_current = call("eglMakeCurrent", uint, pointer, pointer, pointer, pointer)
    get_string = c.CFUNCTYPE(c.c_char_p, uint)(proc(b"glGetString"))
    get_error = call("eglGetError", uint)
    egl_none, platform_device, open_gl = 0x3038, 0x313F, 0x30A2

    devices = (pointer * 16)()
    count = integer()
    assert query_devices(len(devices), devices, c.byref(count)), hex(get_error())
    failures = []
    for device in devices[:count.value]:
        display = get_display(platform_device, device, None)
        major, minor = integer(), integer()
        if not display or not initialize(display, c.byref(major), c.byref(minor)):
            failures.append(hex(get_error()))
            continue
        try:
            assert bind_api(open_gl), hex(get_error())
            config, n = pointer(), integer()
            config_attributes = (integer * 5)(0x3033, 1, 0x3040, 8, egl_none)
            assert choose(display, config_attributes, c.byref(config), 1, c.byref(n)) and n.value, hex(get_error())
            surface = create_surface(display, config, (integer * 5)(0x3057, 8, 0x3056, 8, egl_none))
            context = create_context(display, config, None, (integer * 5)(0x3098, 4, 0x30FB, 3, egl_none))
            assert surface and context and make_current(display, surface, surface, context), hex(get_error())
            facts = dict(renderer=get_string(0x1F01).decode(), vendor=get_string(0x1F00).decode(),
                         version=get_string(0x1F02).decode(), egl_version=f"{major.value}.{minor.value}")
            if facts["vendor"] == "NVIDIA Corporation":
                return facts
            failures.append(facts)
        finally:
            make_current(display, None, None, None)
            terminate(display)
    raise RuntimeError(f"No NVIDIA OpenGL context among {count.value} EGL devices: {failures}")


if __name__ == "__main__":
    print(json.dumps(renderer()))
