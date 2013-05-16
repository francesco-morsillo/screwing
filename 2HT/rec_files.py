




#com_file = open("/tmp/com.dat","r")
pos_file = open("/tmp/position.dat","r")

com = com_file.read(32)

pippo = map(float,com)
#pippo = float(com[0])

print "com: "+com

print "pippo: "+str(pippo)






