import argparse
import base64
import ctypes
from ctypes import wintypes
import json
import os
import time


def notify(message):
    if os.name != 'nt':
        raise OSError('Run this helper with Windows Python')
    shell = ctypes.WinDLL('shell32', use_last_error=True)
    user = ctypes.WinDLL('user32', use_last_error=True)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)

    class NotifyIconData(ctypes.Structure):
        _fields_ = [('cbSize', wintypes.DWORD), ('hWnd', wintypes.HWND),
                    ('uID', wintypes.UINT), ('uFlags', wintypes.UINT),
                    ('uCallbackMessage', wintypes.UINT), ('hIcon', wintypes.HICON),
                    ('szTip', wintypes.WCHAR * 128), ('dwState', wintypes.DWORD),
                    ('dwStateMask', wintypes.DWORD), ('szInfo', wintypes.WCHAR * 256),
                    ('uTimeoutOrVersion', wintypes.UINT), ('szInfoTitle', wintypes.WCHAR * 64),
                    ('dwInfoFlags', wintypes.DWORD), ('guidItem', ctypes.c_ubyte * 16),
                    ('hBalloonIcon', wintypes.HICON)]

    kernel.GetModuleHandleW.argtypes = [wintypes.LPCWSTR]
    kernel.GetModuleHandleW.restype = wintypes.HMODULE
    user.CreateWindowExW.argtypes = [wintypes.DWORD, wintypes.LPCWSTR, wintypes.LPCWSTR,
                                   wintypes.DWORD, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                   ctypes.c_int, wintypes.HWND, wintypes.HMENU,
                                   wintypes.HINSTANCE, wintypes.LPVOID]
    user.CreateWindowExW.restype = wintypes.HWND
    user.LoadIconW.argtypes = [wintypes.HINSTANCE, ctypes.c_void_p]
    user.LoadIconW.restype = wintypes.HICON
    shell.Shell_NotifyIconW.argtypes = [wintypes.DWORD, ctypes.POINTER(NotifyIconData)]
    shell.Shell_NotifyIconW.restype = wintypes.BOOL
    user.DestroyWindow.argtypes = [wintypes.HWND]
    user.PeekMessageW.argtypes = [ctypes.POINTER(wintypes.MSG), wintypes.HWND,
                                 wintypes.UINT, wintypes.UINT, wintypes.UINT]
    user.TranslateMessage.argtypes = [ctypes.POINTER(wintypes.MSG)]
    user.DispatchMessageW.argtypes = [ctypes.POINTER(wintypes.MSG)]
    user.DispatchMessageW.restype = ctypes.c_ssize_t
    window = user.CreateWindowExW(0, 'STATIC', 'RMUL storage alert', 0, 0, 0, 1, 1,
                                 None, None, kernel.GetModuleHandleW(None), None)
    if not window:
        raise ctypes.WinError(ctypes.get_last_error())
    data = NotifyIconData()
    data.cbSize = ctypes.sizeof(data)
    data.hWnd = window
    data.uID = 1
    data.uFlags = 2 | 4
    data.hIcon = user.LoadIconW(None, ctypes.c_void_p(32515))
    data.szTip = 'RMUL storage protection'
    added = False
    try:
        if not shell.Shell_NotifyIconW(0, ctypes.byref(data)):
            raise OSError('Windows declined notification icon registration')
        added = True
        data.uFlags = 16
        data.szInfo = message[:255]
        data.szInfoTitle = 'RMUL disk space alert'
        data.dwInfoFlags = 2
        if not shell.Shell_NotifyIconW(1, ctypes.byref(data)):
            raise OSError('Windows declined the desktop notification request')
        print(json.dumps({'native_notification_request_accepted': True,
                          'visible_to_user': 'not_verified'}, ensure_ascii=False), flush=True)
        until = time.monotonic() + 6
        message_event = wintypes.MSG()
        while time.monotonic() < until:
            while user.PeekMessageW(ctypes.byref(message_event), None, 0, 0, 1):
                user.TranslateMessage(ctypes.byref(message_event))
                user.DispatchMessageW(ctypes.byref(message_event))
            time.sleep(0.05)
    finally:
        if added:
            shell.Shell_NotifyIconW(2, ctypes.byref(data))
        user.DestroyWindow(window)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--message-base64', required=True)
    args = parser.parse_args()
    notify(base64.b64decode(args.message_base64).decode('utf-8'))
