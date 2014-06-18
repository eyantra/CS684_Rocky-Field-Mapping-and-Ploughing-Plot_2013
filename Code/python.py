from subprocess import call
frdr = file('values.txt','r') #Read Log File
fwtr = file('gnu.txt','w+') #Write Coords in Required format

while True:
    line = frdr.readline()

    if len(line) == 0:
        break;

    row = line.split(' ')

    if row[0].isdigit() :

        counter = 0
        
        while counter < len(row)-1:
            s = row[counter]+'  '+row[counter+1]+'\n'
            fwtr.write(s)
            counter += 2
    

frdr.close()#Close log File
fwtr.close()#Close Coords File

call(["gnuplot","C:\\Users\\Pankaj\\Desktop\\plot 1.plt"]) #Plot the Coords into map.
#Here absolute path is given. For reusability of this script One need to change
#this path or can give relative path. In windows make sure to add Environment variable path 
#for GNUPLOT. Otherwise it will give error. 
