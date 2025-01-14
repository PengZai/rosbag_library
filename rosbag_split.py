import rosbag
import rospy
from datetime import datetime

# Path to the original bag file
input_bag_path = 'mergebag_SlowBag_ZED2_and_SlowRun.bag'


# Number of parts to split the bag into
num_parts = 10

# Open the input bag and calculate the splitting parameters
with rosbag.Bag(input_bag_path, 'r') as input_bag:
    total_duration = input_bag.get_end_time() - input_bag.get_start_time()
    split_duration = total_duration / num_parts
    start_time = input_bag.get_start_time()

    # Iterate to create smaller bag files
    for i in range(num_parts):
        output_bag_path = f'split_part_{i + 1}.bag'
        part_start_time = start_time + i * split_duration
        part_end_time = part_start_time + split_duration
        
        with rosbag.Bag(output_bag_path, 'w') as output_bag:
            for topic, msg, t in input_bag.read_messages(
                start_time=rospy.Time.from_sec(part_start_time),
                end_time=rospy.Time.from_sec(part_end_time)
            ):
                datetime_obj = datetime.fromtimestamp(t.to_sec())
                print(f"t:{datetime_obj}, processing {topic}")
                output_bag.write(topic, msg, t)
        
        print(f"Saved part {i + 1} to {output_bag_path}")
