
objname = ""
subobj = 0
faces = 0

with open("model.obj",'r') as openfileobject:
	lines = 0
	for line in openfileobject:
		lines = lines+1
		if 'f' in line:
			faces = faces+1
		if 'o' in line:
			subobj = 0
			faces = 0
			name = line.strip()
			name = name.strip('o')
			objname = name
			#print(name)
		if 'usemtl' in line:
			if (faces>0):
				print("o ",objname+"_"+str(subobj))
			subobj=subobj+1
		#if (lines  % 8192 ==0):
		print(line,end='')
		#	print (line)
		
print("finished")