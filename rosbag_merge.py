#!/usr/bin/env python

import rosbag
from datetime import datetime
import argparse
import os
import rospy


def merge_rosbag(input_bags, output_bag, end_time=None, save_topics=[], delete_topics=[]):


    # Convert human-readable times to ROS time (seconds since epoch)
    def to_ros_time(time):
        dt = datetime.strptime(time, "%Y-%m-%d %H:%M:%S.%f")
        return dt.timestamp()

    # None mean from eariest for start time, and latest for end time
    ros_end_time = None
    # Handle input time formats
    if end_time:
        end_time = to_ros_time(end_time)
        ros_end_time = rospy.Time.from_sec(end_time)


    with rosbag.Bag(output_bag, 'w') as outbag: 
        for inputbag_path in input_bags:
            print("> Reading bag file: " + inputbag_path)
            final_save_topics  = []
            with rosbag.Bag(inputbag_path, 'r') as inbag:

                all_topics = list(inbag.get_type_and_topic_info()[1].keys())
                if len(save_topics) == 0 and len(delete_topics) == 0:
                    # save all the topics
                    final_save_topics = all_topics
                else:
                    remain_topics = list(set(all_topics) - set(delete_topics))
                    for topic in save_topics:
                        if topic not in remain_topics:
                            raise Exception(f"error, save topic:{topic} doesn't existed in reamin_topics:{remain_topics}")
                        final_save_topics.append(topic)
                    if len(final_save_topics) == 0:
                        final_save_topics = remain_topics
                
                for topic, msg, t in inbag.read_messages(topics=final_save_topics, end_time=ros_end_time):
                    if (args.verbose):
                        datetime_obj = datetime.fromtimestamp(t.to_sec())
                        print(f"t:{datetime_obj}, saving {topic} from {inputbag_path}")
                    outbag.write(topic, msg, t)
                    
      
        print(f"merged bag written to {output_bag}")

if __name__ == "__main__":


    # How to use

    parser = argparse.ArgumentParser(description='merge rosbag could merge many rosbags and filter out the topics you dont want')
    parser.add_argument('--output_path',
                        help='output bag file with topics merged', default="save_bags")
    parser.add_argument('--input_bags', required=True,
                        help='input bag files')
    parser.add_argument('-v', '--verbose', action="store_true", default=False,
                        help='verbose output')
    parser.add_argument('-et', '--end_time', default=None,
                        help='date time or timestamp')
    parser.add_argument('--save_topics', default="",
                        help='the topic you want to remain, empty means all the topics, you should use space to seperate these input topics likes, topic1 topic2 topic3 ...')
    parser.add_argument('--delete_topics', default="",
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

    input_bags = args.input_bags.split(" ")
    output_bag_name = [input_bag.split('/')[-1].split('.')[0] for input_bag in input_bags]
    output_bag = os.path.join(args.output_path, 'merge_'+"_".join(output_bag_name)+'.bag')
    if os.path.isfile(output_bag):
        raise Exception(f"error, output_bag {output_bag} have existed")

    


    merge_rosbag(
        input_bags=input_bags,
        output_bag=output_bag,
        end_time=args.end_time,
        save_topics=save_topics,
        delete_topics=delete_topics,
    )