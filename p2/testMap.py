from robot import Map,Node
import numpy as np

map = Map()


print(map)

map.removeMapLine("x","left")
print(map)

map.removeMapLine("x","right")
print(map)


map.removeMapLine("y","up")
#print(map.getMap(),sep = "\n")
print(map)
map.removeMapLine("y","down")
print(map)

mapList = map.getMap()
sublist=mapList[5]
print(mapList[5][12])


n = Node((1,2))
list=[]
list.append(n)

list[0].pos = (4,4)
print(list[0])
