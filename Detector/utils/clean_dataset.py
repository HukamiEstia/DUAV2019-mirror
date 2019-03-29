import glob
import os
import hashlib
import xml.etree.ElementTree as ET
import pickle
from os import listdir, getcwd
from os.path import join

trainfile = open("./data/train.txt", "w+")

for image_path in glob.iglob("./Dataset/*.jpg"):
    print(image_path[10:-4])
    image_id = image_path[10:-4]
    if not glob.glob("./Dataset/%s.txt" %(image_id)):
        os.remove(image_path)
    else:
        label = "./Dataset/%s.txt" %(image_id)
        image = open(image_path, 'rb').read()
        new_id = hashlib.md5(image).hexdigest()
        os.rename(image_path, "./Dataset/%s.jpg" %(new_id))
        os.rename(label, "./Dataset/%s.txt" %(new_id))
        trainfile.write(image_path[2:] + "\n")
    
for label_path in glob.iglob("./Dataset/*.txt"):
    image_id = label_path[10:-4]
    if not glob.glob("./Dataset/%s.jpg" %(image_id)):
        os.remove(label_path)

