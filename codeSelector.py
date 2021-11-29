import subprocess
process = subprocess.Popen(["python3","outputTest.py"], stdin = subprocess.PIPE, stdout = subprocess.PIPE, stderr = subprocess.PIPE, text = True)
#process.stdin.write("2 3")
#process.stdin.close()
print(process.stdout.read())
process.wait()
print("Completed!")
