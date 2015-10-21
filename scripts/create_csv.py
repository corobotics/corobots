#!/usr/bin/env python

import sys
import os.path

if __name__ == "__main__":

    if len(sys.argv) != 3:
        print "usage: create_csv <database_path> <csv_file>"
        sys.exit(1)

    DB_PATH = sys.argv[1]
    FILE_NAME = sys.argv[2]
    SEPARATOR = ";"

    csvFile = open(FILE_NAME, "w")

    label = 0
    for dirname, dirnames, filenames in os.walk(DB_PATH):
        for subdirname in dirnames:
            subject_path = os.path.abspath(os.path.join(dirname, subdirname))
            for filename in os.listdir(subject_path):
                if filename[0] == ".":
                    continue
                abs_path = "%s/%s" % (subject_path, filename)
                csvFile.write( "%s%s%d%s%s\n" % (abs_path, SEPARATOR, label, SEPARATOR, subdirname) )
            label = label + 1

    csvFile.close()