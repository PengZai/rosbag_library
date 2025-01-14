import rosbag
from datetime import datetime
import argparse
import os
from fnmatch import fnmatchcase
import rospy


def trim_rosbag(input_bag, output_bag, start_time=None, end_time=None, start_time_str=None, end_time_str=None, save_topics=[], delete_topics=[]):
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


    final_save_topics  = []

    # Trim the bag
    with rosbag.Bag(output_bag, 'w') as outbag:
        with rosbag.Bag(input_bag, 'r') as inbag:
            all_topics = list(inbag.get_type_and_topic_info()[1].keys())
            
            if len(save_topics) == 0 and len(delete_topics) == 0:
                # save all the topics
                final_save_topics = all_topics
            else:
                remain_topics = list(set(all_topics) - set(delete_topics))

                for topic in save_topics:
                    if topic not in remain_topics:
                        raise Exception(f"error, topic:{topic} doesn't existed in reamin_topics:{remain_topics}")
                    final_save_topics.append(topic)
                if len(final_save_topics) == 0:
                    final_save_topics = remain_topics

            for topic, msg, t in inbag.read_messages(start_time=rospy.Time.from_sec(start_time), end_time=rospy.Time.from_sec(end_time), topics=final_save_topics):
                datetime_obj = datetime.fromtimestamp(t.to_sec())
                print(f"t:{datetime_obj}, saving {topic}")
                outbag.write(topic, msg, t)

    
        
    print(f"Trimmed bag written to {output_bag}, time from {datetime.fromtimestamp(start_time)} to {datetime.fromtimestamp(end_time)}")



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
    parser.add_argument('-t', '--save_topics', default="",
                        help='the topic you want to remain, empty means all the topics, you should use space to seperate these input topics likes, topic1 topic2 topic3 ...')
    parser.add_argument('-dt', '--delete_topics', default="",
                        help='the topic you dont want to remain, you should use space to seperate these input topics likes, topic1 topic2 topic3 ...')


    args = parser.parse_args()

    save_topics = args.save_topics.split(' ')

    if args.delete_topics == "":
        delete_topics = []
    else:
        delete_topics = args.delete_topics.split(' ')

    if args.save_topics == "":
        save_topics = []
    else:
        save_topics = args.save_topics.split(' ')

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
