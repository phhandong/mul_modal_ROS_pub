#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
from pcd_io import write_pcd
def main():
    parser=argparse.ArgumentParser(); parser.add_argument('input_dir',type=Path); parser.add_argument('output_dir',type=Path); parser.add_argument('--ascii',action='store_true'); args=parser.parse_args()
    for csv in sorted(args.input_dir.glob('*.csv')):
        data=np.loadtxt(csv, delimiter=',', skiprows=1, ndmin=2)
        if data.shape[1] < 4: raise ValueError(f'{csv} must have x,y,z,intensity columns')
        write_pcd(args.output_dir/(csv.stem+'.pcd'), data[:,:4], binary=not args.ascii)
if __name__=='__main__': main()
