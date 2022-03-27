#!/usr/bin/env python3
# coding: utf-8
from typing import Dict, List

import rosbag
import pandas as pd
from scipy.spatial.transform import Rotation as R
import numpy as np


def parse_msg_headers(msg):
    msg_string = str(msg)
    headers = []
    levels = []
    i = -1
    for line in msg_string.splitlines():
        key_value = line.strip().split(":")
        if len(key_value) == 1 and key_value[0] == "-":
            levels.append("arrayindex")
            i += 1
            continue
        if len(key_value) != 2:
            continue
        key = key_value[0].strip()
        value = key_value[1].strip()
        indent_level = (len(line) - len(line.lstrip())) // 2 + 1
        if indent_level <= len(levels):
            for _ in range(len(levels) - indent_level + 1):
                levels.pop()
        if indent_level > len(levels):
            levels.append(key)
        if value != '':
            header = "_".join(levels)
            # numbering of arrays
            header = header.replace("arrayindex", str(i))
            headers.append(header)
    return headers


def parse_msg_values(msg):
    msg_string = str(msg)
    values = []
    for line in msg_string.splitlines():
        key_value = line.strip().split(":")
        if len(key_value) != 2:
            continue
        value = key_value[1].strip()
        if value != '':
            try:
                value = float(value)
            except:
                pass
            values.append(value)
    return values


def parse_bag_generic(bagfile: str, topics: List[str] = None) -> Dict[str, pd.DataFrame]:
    """
    Reads from a ROS bag and converts the chosen topics to pandas dataframes
    :param bagfile: the path of the ROS bag
    :param topics: a list of topics to read from the bag
    :return: a dictionary with keys corresponding to topics and values corresponding to the data in dataframe format
    """
    single_topic = topics is not None and not isinstance(topics, list)
    if single_topic:
        topics = [topics]

    bag = rosbag.Bag(bagfile)

    # get list of all topics in the bag
    all_topics = bag.get_type_and_topic_info()[1].keys()

    if topics is None:
        topics = all_topics
    else:
        topics_to_remove = []
        for topic in topics:
            if topic not in all_topics:
                print("Error: " + topic + " does not exist in bag.")
                topics_to_remove.append(topic)
        for topic in topics_to_remove:
            topics.remove(topic)

    df_dict = {}

    # process each topic
    for topic in topics:
        print("Loading:", topic)
        success = True
        values_list = []
        headers = None
        for _, msg, t in bag.read_messages(topic):
            if headers is None:
                headers = parse_msg_headers(msg)
                headers.insert(0, "t_stamp")
            msg_values = parse_msg_values(msg)
            msg_values.insert(0, t.to_sec())
            values_list.append(msg_values)
            if len(msg_values) != len(headers):
                print("Error processing", topic, ": arrays with varying sizes are not supported")
                success = False
                break
        if success:
            df_dict[topic] = pd.DataFrame(values_list, columns=headers)

    bag.close()

    if single_topic:
        # return the only df instead of a dictionary
        return list(df_dict.values())[0]
    else:
        return df_dict


default_renaming_dict = {
    "twist_twist_linear_": "v",
    "twist_linear_": "v",
    "twist_twist_angular_": "w",
    "twist_angular_": "w",
    "pose_pose_position_": "",
    "pose_position_": "",
    "pose_pose_orientation_": "q",
    "pose_orientation_": "q",
    "header_stamp_secs": "t",
    "_state_": "_"
}


def parse_bag(bagfile: str, topics: List[str] = None, renaming_dict: Dict[str, str] = None):
    """
    Improved version of parse_bag_generic that renames the chosen keys to improve readability, and sets the "t"
     column to the message header time stamp if it exists, or to the reception time stamp if not.
    """
    if renaming_dict is None:
        renaming_dict = default_renaming_dict

    df_dict = parse_bag_generic(bagfile, topics)

    for df in df_dict.values():
        df.loc[:, ["stamp_secs" in header for header in df.columns]] += \
            df.loc[:, ["stamp_nsecs" in header for header in df.columns]].to_numpy() * 1e-9

        if renaming_dict:
            new_headers = []
            for header in df.columns:
                for old_name, new_name in renaming_dict.items():
                    header = header.replace(old_name, new_name)
                new_headers.append(header)
            df.columns = new_headers

        if "t" not in df.columns:
            df["t"] = df["t_stamp"]

        if "qx" in df.columns:
            # convert quaternions to euler angles
            orientation_quat = df[["qx", "qy", "qz", "qw"]].to_numpy()
            orientation = R.from_quat(orientation_quat)

            orientation_euler = orientation.as_euler('zyx')
            euler_angles_names = ["yaw", "pitch", "roll"]
            df[euler_angles_names] = orientation_euler

            # unwrap angles to avoid discontinuous jumps
            for angle_name in euler_angles_names:
                df[angle_name] = np.unwrap(df[angle_name])

    return df_dict
