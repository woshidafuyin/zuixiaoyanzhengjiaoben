import os
import sys
import ctypes

dll_path = sys.argv[1]
seed_hex = sys.argv[2] if len(sys.argv) > 2 else ""

print("DLL_PATH =", dll_path)
print("EXISTS   =", os.path.exists(dll_path))

dll_dir = os.path.dirname(dll_path)
print("DLL_DIR  =", dll_dir)
print("DIR_OK   =", os.path.isdir(dll_dir))

if os.path.isdir(dll_dir):
    os.chdir(dll_dir)
    print("CWD      =", os.getcwd())

dll = ctypes.WinDLL(dll_path)
print("LOAD DLL = OK")

func = dll.GenerateKeyEx
print("EXPORT   = GenerateKeyEx OK")

if not seed_hex:
    print("NO SEED, STOP HERE")
    sys.exit(0)

seed = bytes.fromhex(seed_hex)
print("SEED_LEN =", len(seed))
print("SEED_HEX =", seed.hex())

# 先验证 7 参数 GenerateKeyEx：
# seed, seedSize, level, variant, key, maxKeySize, actualKeySize
func.argtypes = [
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.c_uint32,
    ctypes.c_uint32,
    ctypes.c_char_p,
    ctypes.POINTER(ctypes.c_ubyte),
    ctypes.c_uint32,
    ctypes.POINTER(ctypes.c_uint32),
]
func.restype = ctypes.c_int

seed_arr = (ctypes.c_ubyte * len(seed))(*seed)
key_buf = (ctypes.c_ubyte * 255)()
actual_len = ctypes.c_uint32(0)

try:
    ret = func(
        seed_arr,
        len(seed),
        0x11,
        b"",
        key_buf,
        255,
        ctypes.byref(actual_len),
    )

    raw_key = bytes(key_buf[:actual_len.value])

    print("RET      =", ret)
    print("KEY_LEN  =", actual_len.value)
    print("KEY_HEX  =", raw_key.hex())

    if ret != 0:
        sys.exit(4)
    if actual_len.value == 0:
        sys.exit(5)

except Exception as e:
    print("CALL DLL = FAIL")
    print("ERROR    =", repr(e))
    sys.exit(3)