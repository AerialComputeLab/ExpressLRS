Import("env")

from platformio.builder.tools.piotest import ConfigureTestTarget

# `pio test` defines UNIT_TEST (deprecated by PlatformIO in favour of PIO_UNIT_TESTING). ELRS uses
# UNIT_TEST to mean "host-side native test" and compiles the real drivers out, but the RP2040
# on-device tests need them. Drop UNIT_TEST right after PlatformIO adds it, and keep PIO_UNIT_TESTING.
# PlatformIO calls this method on a clone of env, so it must act on the env it is given.
#
# The tests don't use the web server. In test builds PlatformIO compiles lib_deps without resolving
# their dependencies, so ESPAsyncWebServer would fail to find arduino-pico's WiFi headers. It runs
# before the library builders are created, so it can still add them to lib_ignore.
_TEST_LIB_IGNORE = ["ESPAsyncWebServer", "RPAsyncTCP", "WebServer", "WIFI"]

def _configure_device_test_target(env):
    ConfigureTestTarget(env)
    env.Replace(CPPDEFINES=[d for d in env.get("CPPDEFINES", []) if d != "UNIT_TEST"])
    config = env.GetProjectConfig()
    section = "env:" + env["PIOENV"]
    config.set(section, "lib_ignore", env.GetProjectOption("lib_ignore", []) + _TEST_LIB_IGNORE)

env.AddMethod(_configure_device_test_target, "ConfigureTestTarget")
