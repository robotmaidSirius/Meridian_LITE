# Reference:
#   https://docs.platformio.org/en/latest/manifests/library-json/fields/build/extrascript.html
Import("env")

if env.IsIntegrationDump():
   # stop the current script execution
   Return()
print("=====================================================")

# Update submodules
env.Execute("git submodule update --init --recursive")

print("=====================================================")
