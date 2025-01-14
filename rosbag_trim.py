import rosbag
from datetime import datetime
import argparse
import os
from fnmatch import fnmatchcase



def trim_rosbag(input_bag, output_bag, start_time=None, end_time=None, start_time_str=None, end_time_str=None, save_topics='*', delete_topics=None):
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

    nums_skipped_msg = 0
    nums_saved_msg = 0

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
                    if any(fnmatchcase(topic, pattern) for pattern in save_topics) and not any(fnmatchcase(topic, pattern) for pattern in delete_topics) :
                        nums_saved_msg+=1
                        print(f"t:{datetime_obj}, saving {topic}, saved number = {nums_saved_msg}")
                        outbag.write(topic, msg, t)
                    else:
                        nums_skipped_msg+=1
                        print(f"t:{datetime_obj}, skipped {topic}, skipped number = {nums_skipped_msg}")


                if t.to_sec() > end_time:
                    break

    print(f"Trimmed bag written to {output_bag}, time from {datetime.fromtimestamp(start_time)} to {datetime.fromtimestamp(end_time)},\n saved {nums_saved_msg} messages, and skipped {nums_skipped_msg} messages ")



if __name__ == '__main__':

    # How to use

    parser = argparse.ArgumentParser(description='trim rosbag according to datetime or timestamp, and filter out the topics you dont want')
    parser.add_argument('--output_path',
                        help='output bag file with topics merged', default="save_bags")
    parser.add_argument('--input_bag', required=True,
                        help='input bag files')
    parser.add_argument('-st', '--start_time', required=True,
                        help='date time or timestamp')
    parser.add_argument('-et', '--end_time', required=True,
                        help='date time or timestamp')
    parser.add_argument('-v', '--verbose', action="store_true", default=False,
                        help='verbose output')
    parser.add_argument('-t', '--save_topics', default="*",
                        help='the topic you want to remain, * means all the topics, you should use space to seperate these input topics likes, topic1 topic2 topic3 ...')
    parser.add_argument('-dt', '--delete_topics', default="",
                        help='the topic you dont want to remain, you should use space to seperate these input topics likes, topic1 topic2 topic3 ...')


    args = parser.parse_args()

    save_topics = args.save_topics.split(' ')

    if args.delete_topics == "":
        delete_topics = []
    else:
        delete_topics = args.delete_topics.split(' ')

    try:
        os.makedirs(args.output_path)
        print("Directory '%s' created successfully" % args.output_path)
    except OSError as error:
        print("Directory '%s' can not be created or has existed" % args.output_path)

    output_bag = os.path.join(args.output_path, 'trim_'+args.input_bag.split('/')[-1])

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
        input_bag=args.input_bag,
        output_bag=output_bag,
        start_time_str=args.start_time,
        end_time_str=args.end_time,
        save_topics=save_topics,
        delete_topics=delete_topics
    )
