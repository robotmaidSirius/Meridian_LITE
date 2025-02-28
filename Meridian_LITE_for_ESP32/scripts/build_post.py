# Reference:
#   https://docs.platformio.org/en/latest/manifests/library-json/fields/build/extrascript.html
Import("env")
import os

if env.IsIntegrationDump():
   # stop the current script execution
   Return()

# Custom action for post program build
def post_program_action_certutil(source, target, env):
    print("=====================================================")
    program_path = target[0].get_abspath()
    env.Execute("certutil -hashfile " + program_path + " MD5")
    print("=====================================================")

def post_program_action_md5sum(source, target, env):
    print("=====================================================")
    program_path = target[0].get_abspath()
    env.Execute("md5sum " + program_path)
    print("=====================================================")

print("OS name:", os.name)
if os.name == 'nt':
   print(" * call exec: certutil")
   env.AddPostAction("$PROGPATH", post_program_action_certutil)
else:
   print(" * call exec: md5sum")
   env.AddPostAction("$PROGPATH", post_program_action_md5sum)



