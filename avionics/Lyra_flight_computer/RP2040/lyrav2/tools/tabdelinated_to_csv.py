inputfile = ""

file =  open("input.txt","r+")
inputfile = file.read();
inputfile = inputfile.replace(" ",",")
inputfile = inputfile.replace(",,",",")
inputfile = inputfile.replace(",,",",")
inputfile = inputfile.replace(",,",",")
file.close()

outputfile = open("output.csv","w+")
outputfile.write(inputfile);
outputfile.close()