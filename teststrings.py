#! /usr/bin/python

def check_input(stri):
	a = 'abcd'
	b = ''.join(['a', 'b', 'c', 'd'])
	if stri is a:
		print 1
	if stri == a:
		print 2
	if stri == b:
		print 3
	if stri is b:
		print 4
	else:
		print 5

def check_list(what):
	i = 0
	for i in range(what.__len__()):
		print what[i]


string = 'abcd'
check_input(string)
i = 0
while True:
	print '*'
	if i == 10:
		break
	i+=1
listaya = [0, 1, 2, 3, 4]
check_list(listaya)