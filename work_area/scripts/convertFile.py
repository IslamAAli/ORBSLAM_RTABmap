def convertFileFormat(FileIn_path, FileOut_path):
    fin = open(FileIn_path, "r")
    fout = open(FileOut_path, "w")
    for line in fin:
        if line[0] != "%":   #ignore the first line with the header
            splitted_line = line.split(',')
            fout.write(str(float(splitted_line[0]) / (10**9)) + " "
                   + splitted_line[4] + " "
                   + splitted_line[5] + " "
                   + splitted_line[6] + " "
                   + splitted_line[7] + " "
                   + splitted_line[8] + " "
                   + splitted_line[9] + " "
                   + splitted_line[10])

            # print( str(float(splitted_line[0]) / (10**9)) + " "
            #        + splitted_line[4] + " "
            #        + splitted_line[5] + " "
            #        + splitted_line[6] + " "
            #        + splitted_line[7] + " "
            #        + splitted_line[8] + " "
            #        + splitted_line[9] + " "
            #        + splitted_line[10])

    fin.close()
    fout.close()

if __name__ == '__main__':
    convertFileFormat("orbslam2_large_with_loop_pose.txt", "orbslam2_large_with_loop_pose_converted.txt")
    convertFileFormat("orbslam2_long_office_household_pose.txt", "orbslam2_long_office_household_pose_converted.txt")
    convertFileFormat("orbslam2_pioneer_slam_pose.txt", "orbslam2_pioneer_slam_pose_converted.txt")