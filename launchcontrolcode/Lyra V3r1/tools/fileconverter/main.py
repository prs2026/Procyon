inputfile = open("inputfile.csv")
content = inputfile.read()

filecontent = content.replace("\n","},\n{")

inputfile.close()

outputfile = open("outputfile.txt",'w')
outputfile.truncate(0)
outputfile.write("float hitldata[][] = {{")
outputfile.write(filecontent)
outputfile.close()