#!/usr/bin/env python
import os
import yaml
import argparse
from typing import Dict, Any, Optional
ROBOT_ID_WORD_LENGTH = 8

def change_dict_key_name(data: Dict[str, Any], rc_value = "rc_21227") -> Optional[str]:
    keys = [word for word in data.keys() if word.startswith("rc")]
    if not keys:
        return None
    old_id = keys[0]
    tags = [key for key in data.keys() if "rc" in key]
    for tag in tags:
        index = tag.find("rc")
        new_tag = rc_value + tag[index+ROBOT_ID_WORD_LENGTH:] 
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
    if old_id is None:
        print("Unable to find robot id")
    try:
        display_camera_string = file_data["Visualization Manager"]["Displays"][4]["Topic"]["Value"]
        new_string = display_camera_string.replace(old_id, robot_id)
        file_data["Visualization Manager"]["Displays"][4]["Topic"]["Value"] = new_string
    except (IndexError, KeyError) as e:
        print(f"Error: {args.file_path} does not have keys - {e}")
        return None

    with open(args.file_path, 'w') as out_file:
        yaml.dump(file_data, out_file, default_flow_style=False)

if __name__ == "__main__":
    main()
