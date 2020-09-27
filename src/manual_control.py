import configparser
import numpy as np
from arguments import get_args

def reload_config(filename, args):
    manual_control = configparser.ConfigParser()
    manual_control.read("manual_control.ini")
    manual_control = manual_control['manual_control']
    for name, value in manual_control.items():
        value = float(value)
        old_value = -np.inf
        if name in args.__dict__:
            old_value = getattr(args, name)
        if type(old_value) is int:
            value = int(value)
        if old_value != value:
            print(f"Changing args.{name} from {old_value} to {value}.")
            setattr(args, name, value)
    
    return args

if __name__ == "__main__":
    args = get_args()
    args = reload_config("manual_control.ini", args)
    print(args.clear_replay_buffer)