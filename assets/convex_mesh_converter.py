import pybullet as p
import pybullet_data as pd
import os
import argparse

def main(args):
    p.connect(p.DIRECT)
    name_in = args.input
    name_out = args.output
    name_log = args.log

    p.vhacd(name_in, name_out, name_log, alpha=float(args.alpha), resolution=int(args.resolution))
    
if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Convert a non-convex .obj file to multiple convex objects (but still in one single .obj file)')
    parser.add_argument('--input',help='File path for the .obj file you want to convert.')
    parser.add_argument('--output',help='File path for the generated .obj file.')
    parser.add_argument('--log',help='Path for log file. log.txt as default', default='log.txt')
    parser.add_argument('--alpha',default=0.04)
    parser.add_argument('--resolution',metavar='R',default=50000)
    args = parser.parse_args()
    main(args)
