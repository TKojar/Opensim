from pathlib import Path
import numpy as np


def load_mot(fname):
    fname = Path(fname)

    with fname.open("r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    endheader_idx = None
    for i, line in enumerate(lines):
        if line.strip().lower() == "endheader":
            endheader_idx = i
            break

    if endheader_idx is None:
        raise ValueError(f"'endheader' not found in MOT file: {fname}")

    headers = lines[endheader_idx + 1].strip().split()

    data_rows = []
    for line in lines[endheader_idx + 2:]:
        s = line.strip()
        if not s:
            continue
        data_rows.append([float(x) for x in s.split()])

    data = np.array(data_rows, dtype=float)
    return data, headers