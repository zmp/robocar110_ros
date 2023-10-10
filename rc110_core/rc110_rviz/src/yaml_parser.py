#!/usr/bin/env python
import os
import yaml
import argparse
from typing import Dict, Any


def change_dict_key_name(data: Dict[str, Any], rc_value = "rc_21227") -> str:
    old_id = [word for word in data.keys() if "rc" in word][0]
    tags = [key for key in data.keys() if "rc" in key]
    for tag in tags:
        index = tag.find("rc")
        new_tag = rc_value + tag[index+8:] 
        data[new_tag] = data.pop(tag)
     
    return old_id

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--file_path", "-f", help="rviz file to be changed.", required=True)
    args = parser.parse_args()
    robot_id = os.uname()[1].replace('-', '_')
    with open(args.file_path, 'r') as file:
        file_data = yaml.safe_load(file)

    data = file_data["Visualization Manager"]["Displays"][3]["Links"]

    old_id = change_dict_key_name(data, robot_id)
    display_camera_string = file_data["Visualization Manager"]["Displays"][4]["Topic"]["Value"]
    new_string = display_camera_string.replace(old_id, robot_id)
    file_data["Visualization Manager"]["Displays"][4]["Topic"]["Value"] = new_string

    with open(args.file_path, 'w') as out_file:
        yaml.dump(file_data, out_file, default_flow_style=False)

if __name__ == "__main__":
    main()
