Import("env")
import os

# フラッシュ末尾はメッセージ・設定の保存領域（FLASH_EEP: 64B x 5ページ）
FLASH_TOTAL = 16384
EEP_RESERVED = 64 * 5

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(
        " ".join([
            "$OBJCOPY", "-O", "binary",
            "$BUILD_DIR/${PROGNAME}.elf",
            "$BUILD_DIR/${PROGNAME}.bin"
        ]),
        "Building $BUILD_DIR/${PROGNAME}.bin"
    )
)


def check_eep_overlap(target, source, env):
    bin_path = env.subst("$BUILD_DIR/${PROGNAME}.bin")
    size = os.path.getsize(bin_path)
    limit = FLASH_TOTAL - EEP_RESERVED
    print("Program size: %d / %d bytes (free %d, EEPROM area %d bytes excluded)"
          % (size, limit, limit - size, EEP_RESERVED))
    if size > limit:
        print("ERROR: program overlaps the FLASH_EEP save area")
        env.Exit(1)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", check_eep_overlap)
