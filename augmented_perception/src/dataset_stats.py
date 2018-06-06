import sys

with open(sys.argv[1]) as f:
    content = f.readlines()

content = [x.strip() for x in content] 
ids = []
objects = {'car': 0, 'van': 0, 'people': 0, 'bicycle': 0, 'sign': 0, 'misc': 0, 'DontCare': 0}
for line in content:
	if " " in line:
		asd = line.split()
		if asd[5] != 'ID':
			if asd[5] not in ids:
				objects[asd[4]]+=1
			ids.append(asd[5])


print("car: ",objects['car'])
print("van: ",objects['van'])
print("people: ",objects['people'])
print("bicycle: ",objects['bicycle'])
print("sign: ",objects['sign'])
print("misc: ",objects['misc'])
print("DontCare: ",objects['DontCare'])