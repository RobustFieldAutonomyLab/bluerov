import subprocess

cmd = [
    'socat pty,link=/tmp/vn100_imu,waitslave,raw,echo=0,ignoreeof tcp:192.168.2.2:14661'
]
process = subprocess.Popen(cmd[0].split(), stdout=subprocess.PIPE)
output, error = process.communicate()
