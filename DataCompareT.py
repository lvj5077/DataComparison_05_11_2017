import linecache
import numpy


def quaternion_to_transform_matrix(x, y, z, qx, qy, qz, qw):
    Transform = numpy.identity(4)
    Transform = [[(1 - 2 * (qy**2. + qz**2.)),    (2 * (qx * qy - qz * qw)),     (2 * (qx * qz + qy * qw)),      x],
                 [(2 * (qx * qy + qz * qw)),      (1 - 2 * (qx**2. + qz**2.)),   (2 * (qy * qz - qx * qw)),      y],
                 [(2 * (qx * qz - qy * qw)),      (2 * (qy * qz + qx * qw)),     (1 - 2 * (qx**2. + qy**2.)),    z],
                 [0,                               0,                             0,                             1]]
    return Transform

data_dir = '/Users/Q/Desktop/DataComparison_05_11_2017/CameraTrajectoryOrg.txt'
truth_dir = '/Users/Q/DataSets/ORB_test_Data/rgbd_dataset_freiburg1_desk/groundtruth.txt'

output_file_name = '/Users/Q/Desktop/result.txt'

output_file = open(output_file_name, 'wb')

disDt = disTt = 0
error = 0
lineN = 4
t_diff = 10
MtR = MdR = numpy.identity(3)
Mtt = Mdt = [0, 0, 0]
MtT_now = MdT_now = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]
xt = yt = zt = qxt = qyt = qzt = qwt = 0
xd = yd = zd = qxd = qyd = qzd = qwd = 0

firstT = 1

seeN = 3
seeI = 0

with open(data_dir, 'rb') as myData:
    for line in myData:
        tokenD = line.split(' ')
        timestampD = float(tokenD[0])
        lineT = linecache.getline(truth_dir, lineN)
        tokenT = lineT.split(' ')
        timestampT = float(tokenT[0])
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
            qxt = float(tokenT[4])
            qyt = float(tokenT[5])
            qzt = float(tokenT[6])
            qwt = float(tokenT[7])
            MtT_last = MtT_now
            MtT_now = numpy.array( quaternion_to_transform_matrix(xt, yt, zt, qxt, qyt, qzt, qwt) )
            delta_t = numpy.dot(MtT_now, numpy.linalg.pinv(MtT_last))
            MtR = delta_t[0:3,0:3]
            Mtt = delta_t[0:3,3]
            Mttt = (MtT_now-MtT_last)[0:3,3]
        if (firstT > 0):
            firstT = 0
            disD = 0
            disT = 0
            disTt = 0
        else:
            # disD = (xd - float(tokenD[1]))**2. + (yd - float(tokenD[2]))**2. + (zd - float(tokenD[3]))**2.
            # disD = disD**(1. / 2)
            # disT = numpy.linalg.norm(Mttt)
            disT = numpy.linalg.norm(Mtt)
            disTt = disT + disTt

        xd = float(tokenD[1])
        yd = float(tokenD[2])
        zd = float(tokenD[3])
        qxd = float(tokenD[4])
        qyd = float(tokenD[5])
        qzd = float(tokenD[6])
        qwd = float(tokenD[7])
        MdT_last = MdT_now
        MdT_now = numpy.array( quaternion_to_transform_matrix(xd, yd, zd, qxd, qyd, qzd, qwd) )
        delta_d = numpy.dot(MdT_now, numpy.linalg.pinv(MdT_last))
        MdR = delta_d[0:3,0:3]
        Mdt = delta_d[0:3,3]
        Mdtt = (MdT_now-MdT_last)[0:3,3]
        # disD = numpy.linalg.norm(Mdtt)
        disD = numpy.linalg.norm(Mdt)
        disDt = disD + disDt

        print ("~~~~~~~~~~t-t~~~~~~~~~~~~~~~~")
        print Mdtt
        print numpy.linalg.norm(Mdtt)
        print ("~~~~~~~~~~t->t~~~~~~~~~~~~~~~~")
        print Mdt
        print numpy.linalg.norm(Mdt)
        print ("~~~~~~~~~~~delta~~~~~~~~~~~~~~")
        print delta_d
        print ("~~~~~~~~~~~R???~~~~~~~~~~~~~")
        print numpy.linalg.norm(MdR,1)
        print ("~~~~~~~~~~~errorL~~~~~~~~~~~~~~")
        errorL = numpy.dot(delta_d, numpy.linalg.pinv(delta_t))
        print errorL
        errorL = numpy.linalg.norm(errorL)
        print errorL
        print ("~~~~~~~~~~~errorLD~~~~~~~~~~~~~~")
        errorLD = abs(disD - disT)
        print errorLD
        print ("==============================")
        error = error + errorLD
        # output_file.write(str(disDt) + " " + str(disTt) + " " +  str(errorLD) + "\n")
        output_file.write(str(disD) + " "  + str(disT) + " " + str(errorLD) + "\n")

        seeI = seeI+1
        # if seeI>seeN:
        #     break

    print(error)

output_file.close()
