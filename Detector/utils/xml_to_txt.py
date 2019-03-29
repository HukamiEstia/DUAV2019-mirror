import xml.etree.ElementTree as ET
import pickle
import os
from os import listdir, getcwd
from os.path import join
import glob
import hashlib

trainfile = open("./data/train.txt", "w+")

classes = ["croix_rouge",
"croix_jaune",
"croix_bleue",
"barre_rouge",
"barre_jaune",
"barre_bleue"]

def convert(size, box):
    dw = 1./(size[0])
    dh = 1./(size[1])
    x = (box[0] + box[1])/2.0 - 1
    y = (box[2] + box[3])/2.0 - 1
    w = box[1] - box[0]
    h = box[3] - box[2]
    x = x*dw
    w = w*dw
    y = y*dh
    h = h*dh
    return (x,y,w,h)

def convert_annotation(image_id):
    in_file = open('./Dataset/%s.xml'%(image_id))
    out_file = open('./Dataset/%s.txt'%(image_id), 'w')
    tree=ET.parse(in_file)
    root = tree.getroot()
    size = root.find('size')
    w = int(size.find('width').text)
    h = int(size.find('height').text)

    for obj in root.iter('object'):
        difficult = obj.find('difficult').text
        cls = obj.find('name').text
        if cls not in classes or int(difficult)==1:
            continue
        cls_id = classes.index(cls)
        xmlbox = obj.find('bndbox')
        b = (float(xmlbox.find('xmin').text), float(xmlbox.find('xmax').text), float(xmlbox.find('ymin').text), float(xmlbox.find('ymax').text))
        bb = convert((w,h), b)
        out_file.write(str(cls_id) + " " + " ".join([str(a) for a in bb]) + '\n')

for image_path in glob.iglob("./Dataset/*.jpg"):
    print(image_path[10:-4])
    image_id = image_path[10:-4]
    if glob.glob("./Dataset/%s.xml" %(image_id)):
        convert_annotation(image_id)
        os.remove("./Dataset/%s.xml" %(image_id))
    if glob.glob("./Dataset/%s.txt" %(image_id)):
        label = "./Dataset/%s.txt" %(image_id)
        image = open(image_path, 'rb').read()
        new_id = hashlib.md5(image).hexdigest()
        os.rename(image_path, "./Dataset/%s.jpg" %(new_id))
        os.rename(label, "./Dataset/%s.txt" %(new_id))
        trainfile.write(image_path[2:] + "\n")        
    else:
        os.remove(image_path)

for label_path in glob.iglob("./Dataset/*.txt"):
    image_id = label_path[10:-4]
    if not glob.glob("./Dataset/%s.jpg" %(image_id)):
        os.remove(label_path)

for xml_path in glob.iglob("./Dataset/*.xml"):
    os.remove(xml_path)