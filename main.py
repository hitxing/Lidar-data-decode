import argparse
import yaml

from lidar_manager import *

GPS=False

def read_params(path):
    try:
        f = open(path, 'rb')
    except Exception as ex:
        print(str(ex))

    params = yaml.load(f.read(), Loader=yaml.FullLoader)
    return params

def main(args):
    path = args['path']
    lidar_type = args['type']
    outdir = args['out_dir']
    config = args['config']

    params = read_params(config)

    if "velodyne" in lidar_type.lower():
        lidar_manager = VelodyneManager(lidar_type, path, outdir, params)
    elif "robosense" in lidar_type.lower():
        lidar_manager = RoboSenseManager(lidar_type, path, outdir, params)

    lidar_manager.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--path', help="Path to the pcap file", required=True)
    parser.add_argument('-o', '--out-dir', help="Path to the output directory", required=True)
    parser.add_argument('-c', '--config', help="Path of the configuration file", required=True)
    parser.add_argument('-t', '--type', help="Lidar name", required=False, default="velodyneVLP16")

    args = vars(parser.parse_args())
    main(args)