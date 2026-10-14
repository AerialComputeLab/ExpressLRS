Import("env")

from platformio.builder.tools.piotest import ConfigureTestTarget

# `pio test` defines UNIT_TEST (deprecated by PlatformIO in favour of PIO_UNIT_TESTING). ELRS uses
# UNIT_TEST to mean "host-side native test" and compiles the real drivers out, but the RP2040
# on-device tests need them. Drop UNIT_TEST right after PlatformIO adds it, and keep PIO_UNIT_TESTING.
# PlatformIO calls this method on a clone of env, so it must act on the env it is given.
def _configure_device_test_target(env):
    ConfigureTestTarget(env)
    env.Replace(CPPDEFINES=[d for d in env.get("CPPDEFINES", []) if d != "UNIT_TEST"])

env.AddMethod(_configure_device_test_target, "ConfigureTestTarget")
