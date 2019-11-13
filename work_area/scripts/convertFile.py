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
    convertFileFormat("example_orbslam_out.txt", "test2.txt")