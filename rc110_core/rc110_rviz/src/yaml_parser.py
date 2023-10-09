#!/usr/bin/env python
import os
import yaml
import getpass
import argparse
from typing import Dict, Any


def change_dict_key_name(data: Dict[str, Any], rc_value = "rc_21227") -> None:
    tags = [key for key in data.keys() if "rc_" in key]
    for tag in tags:
        index = tag.find("rc")
        new_tag = rc_value + tag[index+8:] 
        data[new_tag] = data.pop(tag)
     
    return data

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--file_path", "-f", help="rviz file to be changed.", required=True)
    parser.add_argument("--robot_id", "-r", help="The robot id to change to.", required=True)
    args = parser.parse_args()
    old_id = getpass.getuser()
    
    with open(args.file_path, 'r') as file:
        file_data = yaml.safe_load(file)

    data = file_data["Visualization Manager"]["Displays"][3]["Links"]

    change_dict_key_name(data, args.robot_id)
    display_camera_string = file_data["Visualization Manager"]["Displays"][4]["Topic"]["Value"]
    new_string = display_camera_string.replace(old_id, args.robot_id)
    file_data["Visualization Manager"]["Displays"][4]["Topic"]["Value"] = new_string

    with open(args.file_path, 'w') as out_file:
        yaml.dump(file_data, out_file, default_flow_style=False)

if __name__ == "__main__":
    main()
