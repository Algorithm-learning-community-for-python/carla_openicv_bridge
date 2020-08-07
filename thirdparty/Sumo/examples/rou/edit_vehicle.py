from lxml import etree
rou_xml=etree.parse('/home/yining/CARLA_0.9.9/Co-Simulation/Sumo/examples/rou/Town04.rou.xml')
root=rou_xml.getroot()
print(root.items())
print(root.keys())

print(root.xpath('//vehicle'))
vehicle_list=root.xpath('//vehicle')
print(len(vehicle_list))


#departPos="random" departSpeed="0.00" arrivalPos="random" arrivalSpeed="0.00">
for node in root.xpath('//vehicle'):
    node.set("departPos","random")
    node.attrib["departPos"]="random"
    node.attrib["departSpeed"]="0.00"
    node.attrib["arrivalPos"]="random"
    node.attrib["arrivalSpeed"]="0.00"
tree=etree.ElementTree(root)
print(etree.tostring(rou_xml))
tree.write('/home/yining/CARLA_0.9.9/Co-Simulation/Sumo/examples/rou/Town04_flow.rou.xml',pretty_print=True,xml_declaration=True, encoding='utf-8')