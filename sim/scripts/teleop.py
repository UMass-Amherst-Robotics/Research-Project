#!/usr/bin/env python
import rospy, sys, tty, termios, select
"""
def get_key(key_timeout):
  tty.setraw(sys.stdin.fileno())
  rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
  if rlist:
    key = sys.stdin.read(1)
  else:
    key = ''
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key
"""

def main():
  """
  rospy.loginfo("Starting teleop node")
  rospy.init_node("teleop")
  pub = rospy.Publisher("velocities", 1)
  """
  old_struct = termios.tcgetattr(sys.stdin)
  print(old_struct)

  # Uses noncanonical mode and doesn't print typed characters. See the "termios" manual page.
  tty.setraw(sys.stdin.fileno())
  # Select will block until a character is typed to stdin
  read_list, _, _ = select.select([sys.stdin], [], [])

  try:
    key = sys.stdin.read(1)
  except:
    print("error reading")
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_struct)

if __name__ == "__main__":
  main()
