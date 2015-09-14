#!/usr/bin/env python
import sys
import os
import subprocess as sp
import fcntl

import roslib; roslib.load_manifest("rosshell")
import rospy
from std_msgs.msg import String

import time

def callbackStdIn(msg):
	global process

	# un/comment this to enable tostopic to publish strings...
	msg.data = msg.data.replace("\\n", "\n")

	process.stdin.write(msg.data)
	rospy.loginfo("stdin: " +msg.data)

if __name__ == "__main__":

	rospy.init_node('rosshellX')

	topicStdOut = rospy.get_param('~stdout', "/rosshellX/stdout")
	topicStdErr = rospy.get_param('~stderr', "/rosshellX/stderr")
	topicStdIn  = rospy.get_param('~stdin', "/rosshellX/stdin")
	cmd_user    = rospy.get_param('~command', "")

	#print len(cmd_user)

	# check if there is a command
	if len(cmd_user) == 0:
		if len(sys.argv) >= 2:
			cmd_user = sys.argv[1]
		else:
			rospy.logerr("no command to interprete defined, try _command:=\"command\"")
			sys.exit()

	rospy.loginfo("execute command: " + cmd_user)
	rospy.loginfo("stdout on: " + topicStdOut)
	rospy.loginfo("stderr on:  " + topicStdErr)
	rospy.loginfo("stdin on:  " + topicStdIn)

	# change the buffer mode of the subprocess-shell to linebufferd
	cmd_stdbuf = "stdbuf -o L"
	cmd = cmd_stdbuf + " " + cmd_user

	process = sp.Popen(cmd.split(" "), stdout=sp.PIPE, stderr=sp.PIPE, stdin=sp.PIPE, bufsize=1)
	rospy.loginfo("started process with pid " + str(process.pid))
	rospy.loginfo("XXX")

	pubOUT = rospy.Publisher( topicStdOut, String, queue_size=10)
	pubERR = rospy.Publisher( topicStdErr, String, queue_size=10)
	subIN  = rospy.Subscriber(topicStdIn,  String, callbackStdIn)

	# putting readline of stdout and stderr into non-blocking mode
	fd_out = process.stdout.fileno()
	fl = fcntl.fcntl(fd_out, fcntl.F_GETFL)
	fcntl.fcntl(fd_out, fcntl.F_SETFL, fl | os.O_NONBLOCK)

	fd_err = process.stderr.fileno()
	fl = fcntl.fcntl(fd_err, fcntl.F_GETFL)
	fcntl.fcntl(fd_err, fcntl.F_SETFL, fl | os.O_NONBLOCK)


	while not rospy.is_shutdown():
		try:
			stdout = process.stdout.readline()
			if len(stdout) > 0:
				pubOUT.publish(String(stdout))
				rospy.loginfo("stdout: " +stdout)
		except Exception as e:
			pass

		try:
			stderr = process.stderr.readline()
			if len(stderr) > 0:
				pubERR.publish(String(stderr))
				rospy.logerr("stderr: " +stderr)
		except Exception as e:
			pass

		time.sleep(0.001)

		# process is terminated
		#if process.poll()!=0:
		#	break

	#process.terminate()
	ret = process.returncode
	rospy.signal_shutdown('finish')
	sys.exit(ret)
