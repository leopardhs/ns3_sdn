#! /usr/bin/python 

import xml.etree.ElementTree

e = xml.etree.ElementTree.parse('input.fcd.xml')
r = e.getroot()

for times in r.iter('timestep'):
    thetime = float(times.get('time'))-112
    times.set('time',str(thetime))


e.write('fcd.xml')
    
    
