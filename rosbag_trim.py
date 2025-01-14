import rosbag
from datetime import datetime

def trim_rosbag(input_bag, output_bag, start_time=None, end_time=None, start_time_str=None, end_time_str=None):
    """
    Trims a ROS bag file between a start and end time.

    Parameters:
    - input_bag (str): Path to the input bag file.
    - output_bag (str): Path to the output trimmed bag file.
    - start_time (float): Start time in seconds since epoch (optional).
    - end_time (float): End time in seconds since epoch (optional).
    - start_time_str (str): Start time in human-readable format (YYYY-MM-DD HH:MM:SS.sss) (optional).
    - end_time_str (str): End time in human-readable format (YYYY-MM-DD HH:MM:SS.sss) (optional).
    """

    # Convert human-readable times to ROS time (seconds since epoch)
    def to_ros_time(time_str):
        dt = datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S.%f")
        return dt.timestamp()

    # Handle input time formats
    if start_time_str:
        start_time = to_ros_time(start_time_str)
    if end_time_str:
        end_time = to_ros_time(end_time_str)

    # Check that time range is specified
    if start_time is None or end_time is None:
        raise ValueError("Either timestamps or human-readable times must be specified for start and end.")

    # Trim the bag
    with rosbag.Bag(output_bag, 'w') as outbag:
        with rosbag.Bag(input_bag, 'r') as inbag:
            for topic, msg, t in inbag.read_messages():
                if start_time <= t.to_sec() <= end_time:
                    datetime_obj = datetime.fromtimestamp(t.to_sec())
                    print(f"{i + 1}, processing {topic} data as to {output_bag_path}")
                    outbag.write(topic, msg, t)

                if t.to_sec() > end_time:
                    break

    print(f"Trimmed bag written to {output_bag}")



if __name__ == '__main__':

    # How to use

    # Trim Using Human-Readable Time:
    # trim_rosbag(
    #     input_bag="example.bag",
    #     output_bag="trimmed_human_time.bag",
    #     start_time_str="2024-06-03 12:00:11.80",
    #     end_time_str="2024-06-03 12:24:54.56"
    # )

    # Trim Using Timestamps:
    # trim_rosbag(
    #     input_bag="example.bag",
    #     output_bag="trimmed_timestamp.bag",
    #     start_time=1717412411.80,
    #     end_time=1717413894.56
    # )

    trim_rosbag(
        input_bag="/mnt/d/zhipeng/datasets/Orchard_EMR_Jun_2024/SlowRun.bag",
        output_bag="trimmed_SlowRun_from_12_01_to_12_02.bag",
        start_time_str="2024-06-03 12:01:00.00",
        end_time_str="2024-06-03 12:02:00.00"
    )
