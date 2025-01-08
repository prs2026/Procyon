import sys,time,os

readdata = "testing testing newfile secondfile"
print(readdata)

readtime = str(time.strftime('%m,%d,%Y,%H,%M,%S'))
currentdir = os.path.realpath(__file__)

currentdir = os.path.abspath(os.path.join(currentdir, os.pardir))

path = currentdir + ("\LOGS "+readtime)

print(path)

os.mkdir(path)
rawfilepath = path + "/rawfile.csv"
file = open(rawfilepath,'w+')

file.write(readdata)

file.close()

splitdata = readdata.split("newfile")
flightnum = 1
for x in splitdata:
    x = x.replace("newfile",'')
    logfilepath = path +('/flight '+str(flightnum) + '.csv')
    logfile = open(logfilepath,'w+')
    logfile.write(x)
    logfile.close()

