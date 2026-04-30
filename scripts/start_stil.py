import subprocess

def start_sitl():
    cmd = [
        "make",
        "px4_sitl",
        "gazebo"
    ]
    
    process = subprocess.Popen(cmd)
    return process

if __name__ == "__main__":
    start_sitl()
