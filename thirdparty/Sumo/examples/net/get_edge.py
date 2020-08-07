from lxml import etree
net_xml=etree.parse('/home/yining/CARLA_0.9.9/Co-Simulation/Sumo/examples/net/Town04.net.xml')
root=net_xml.getroot()
print(root.items())
print(root.keys())

print(root.xpath('//edge'))
edge_list=root.xpath('//edge')
print(len(edge_list))

id_list=root.xpath('//edge/@id')
print(id_list)