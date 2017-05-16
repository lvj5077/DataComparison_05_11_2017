import linecache

data_dir = '/Users/Q/Desktop/DataComparison_05_11_2017/CameraTrajectory.txt'
truth_dir = '/Users/Q/DataSets/ORB_test_Data/rgbd_dataset_freiburg1_desk/groundtruth.txt'

output_file_name = '/Users/Q/Desktop/resultT.txt'


output_file = open(output_file_name, 'wb')

error = 0
lineN = 4
t_diff = 10
with open(data_dir, 'rb') as myData:
    printed = 0
    for line in myData:
        tokenD = line.split(' ')
        timestampD = float(tokenD[0])
        lineT = linecache.getline(truth_dir, lineN)
        tokenT = lineT.split(' ')
        timestampT = float(tokenT[0])
        xt = float(tokenT[1])
        yt = float(tokenT[2])
        zt = float(tokenT[3])
        t_diff = 10
        while (abs(timestampT - timestampD) <= t_diff) and lineN < 2338:
            t_diff = abs(timestampT - timestampD)

            lineN = lineN + 1
            lineT = linecache.getline(truth_dir, lineN)
            tokenT = lineT.split(' ')
            timestampT = float(tokenT[0])
            xt = float(tokenT[1])
            yt = float(tokenT[2])
            zt = float(tokenT[3])
            # print (lineN)
            # print (t_diff)
        if (printed == 0):
            # print (timestampT)
            # print (xt)
            # print (yt)
            # print (zt)
            xoff = xt
            yoff = yt
            zoff = zt
            printed = 1
        xd = float(tokenD[1]) + 1.3137
        yd = float(tokenD[2]) + 0.8486
        zd = float(tokenD[3]) + 1.5192

        # print (timestampT)
        errorL = (xd - xt)**2 + (yd - yt)**2 + (zd - zt)**2
        errorL = errorL ** (1. / 2)
        error = error + errorL
        output_file.write(str(errorL) + "\n")

    print(error)

output_file.close()
