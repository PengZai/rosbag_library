#!/usr/bin/env python

import argparse
from fnmatch import fnmatchcase
from datetime import datetime
from rosbag import Bag
import os

def main():

    parser = argparse.ArgumentParser(description='Merge one or more bag files with the possibilities of filtering topics.')
    parser.add_argument('outputbag',
                        help='output bag file with topics merged', default="save_bags/merge_rosbag.bag")
    parser.add_argument('inputbag', nargs='+',
                        help='input bag files')
    parser.add_argument('-v', '--verbose', action="store_true", default=False,
                        help='verbose output')
    parser.add_argument('-t', '--topics', default="*",
                        help='the topic you want to remain, * means all the topics, you should use space to seperate these input topics likes, topic1 topic2 topic3 ...')
    parser.add_argument('-dt', '--delete_topics', default="",
                        help='the topic you dont want to remain, you should use space to seperate these input topics likes, topic1 topic2 topic3 ...')


    args = parser.parse_args()

    topics = args.topics.split(' ')
    delete_topics = args.delete_topics.split(' ')
    output_path = args.outputbag.split('/')[:-1].join('/')
    os.makedirs(output_path)

    total_included_count = 0
    total_skipped_count = 0

    if (args.verbose):
        print("Writing bag file: " + args.outputbag)
        print("Matching topics against patters: '%s'" % ' '.join(topics))

    with Bag(args.outputbag, 'w') as o: 
        for ifile in args.inputbag:
            included_count = 0
            skipped_count = 0
            if (args.verbose):
                print("> Reading bag file: " + ifile)
            with Bag(ifile, 'r') as ib:
                for topic, msg, t in ib:
                    if any(fnmatchcase(topic, pattern) for pattern in topics) and not any(fnmatchcase(topic, pattern) for pattern in delete_topics) :
                        if (args.verbose):
                            datetime_obj = datetime.fromtimestamp(t.to_sec())
                            print(f"t:{datetime_obj}, processing {topic}")
                        o.write(topic, msg, t)
                        included_count += 1
                    else:
                        skipped_count += 1
            total_included_count += included_count
            total_skipped_count += skipped_count
            if (args.verbose):
                print("< Included %d messages and skipped %d" % (included_count, skipped_count))

    if (args.verbose):
        print("Total: Included %d messages and skipped %d" % (total_included_count, total_skipped_count))

if __name__ == "__main__":
    main()