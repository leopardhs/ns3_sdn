#! /usr/bin/python
import xml.etree.ElementTree

e = xml.etree.ElementTree.parse('input.rou.xml')
r = e.getroot()

for times in r.iter('vehicle'):
    deptime = float(times.get('depart'))
    if (deptime < 112):
        deptime = 0
    else:
        deptime -= 112
    times.set('depart',str(deptime))


e.write('rou.xml')
    
    
