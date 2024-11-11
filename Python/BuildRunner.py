import subprocess
import time
import threading
import os

RESTART_INTERVAL = 1800 
BUILD_EXE_PATH = r""
BUILD_ARGS = ["-batchmode", "-nographics"]
NUM_INSTANCES = 6  

os.chdir(r"C:\Users")  # dir

def run_single_build(instance_id):
    while True:
        start_time = time.time()
        
        print(f"Instance {instance_id} starts，Time：{time.strftime('%Y-%m-%d %H:%M:%S')}")
        process = subprocess.Popen([BUILD_EXE_PATH] + BUILD_ARGS, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        
        try:
            while True:
                output = process.stdout.readline()
                if output:
                    print(f"Instance {instance_id}: {output.strip()}")
                
                if process.poll() is not None:
                    break
                
                if time.time() - start_time >= RESTART_INTERVAL:
                    print(f"Instance {instance_id} restarts，Time：{time.strftime('%Y-%m-%d %H:%M:%S')}")
                    process.terminate()
                    time.sleep(5) 
                    break
        
        except KeyboardInterrupt:
            print(f"Instance {instance_id} is closing...")
            process.terminate()
            break

def run_multiple_builds():
    threads = []
    for i in range(NUM_INSTANCES):
        thread = threading.Thread(target=run_single_build, args=(i+1,))
        threads.append(thread)
        thread.start()
        time.sleep(3)
    
    for thread in threads:
        thread.join()

if __name__ == "__main__":
    run_multiple_builds()