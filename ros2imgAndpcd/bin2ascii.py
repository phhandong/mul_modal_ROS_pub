#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
from pcd_io import write_pcd

def read_binary_xyzi(path):
    payload = path.read_bytes(); marker = b'DATA binary\n'; offset = payload.find(marker)
    if offset < 0: raise ValueError(f'{path} is not a binary PCD')
    return np.frombuffer(payload[offset + len(marker):], dtype='<f4').reshape((-1, 4))
def main():
    parser=argparse.ArgumentParser(); parser.add_argument('input_dir',type=Path); parser.add_argument('output_dir',type=Path); args=parser.parse_args()
    for path in sorted(args.input_dir.glob('*.pcd')): write_pcd(args.output_dir/path.name, read_binary_xyzi(path), binary=False)
if __name__=='__main__': main()
