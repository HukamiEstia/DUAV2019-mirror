import glob
import os
import hashlib

testlist = open("./data/test.txt", "w+")

for image_path in glob.iglob("./TestSet/*.jpg"):
    print(image_path)
    image = open(image_path, 'rb').read()
    new_id = hashlib.md5(image).hexdigest()
    os.rename(image_path, "./TestSet/%s.jpg" %(new_id))
    testlist.write( "TestSet/%s.jpg" %(new_id) + "\n")

testlist.close()