# import subprocess, shlex
#
# def subprocess_cmd(command):
#     process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
#     proc_stdout = process.communicate()[0].strip()
#     print proc_stdout

# subprocess_cmd('rosnode list; echo b')


import os
os.system('rosrun test headertest.py ')