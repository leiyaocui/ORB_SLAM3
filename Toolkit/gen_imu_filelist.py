from config import DATA_PATH

input_path = DATA_PATH

imus = sorted((input_path / "imu").glob("*.txt"), key=lambda x: int(x.stem))

def convert_timestamp(timestamp: str):
    timestamp = float(timestamp) / 10e9
    return timestamp

fd = open(input_path / "imu_filelist.txt", "w")
for imu in imus:
    t = convert_timestamp(imu.stem)
    
    print(f"{t} {imu}", file=fd)
fd.close()