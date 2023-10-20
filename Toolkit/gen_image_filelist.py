from config import DATA_PATH

input_path = DATA_PATH

rgbs = sorted((input_path / "rgb").glob("*.png"), key=lambda x: int(x.stem))
depths = sorted((input_path / "depth").glob("*.png"), key=lambda x: int(x.stem))

def convert_timestamp(timestamp: str):
    timestamp = float(timestamp) / 10e9
    return timestamp

fd = open(input_path / "image_filelist.txt", "w")
for rgb, depth in zip(rgbs, depths):
    t = convert_timestamp(rgb.stem)
    
    print(f"{t:.9f} {rgb} {t:.9f} {depth}", file=fd)
fd.close()