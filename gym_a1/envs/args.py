import argparse

def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--direct", type=bool, default="True", 
        help="connect DIRECT, otherwise connect GUI")
    parser.add_argument("--gravity", type=list, default=[0,0,-9.807],
        help="gravity on earth")
    parser.add_argument("--plane", type=str, default="plane.urdf",
        help="urdf file for our environment")
    parser.add_argument("--robot", type=str, default="a1.urdf",
        help="urdf file we load into physics server")
    parser.add_argument("--start-pos", type=list, default=[0,0,5],
        help="default position of our robot when plopped into the environment")
    parser.add_argument("--hip-angle", type=float, default=0,
        help="hip angle")
    parser.add_argument("--thigh-angle", type=float, default=0.9,
        help="thigh angle")
    parser.add_argument("--calf-angle", type=float, default=-1.8,
        help="calf angle")
    parser.add_argument("--num_legs", type=float, default=4,
        help="number of legs")
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    print(args)