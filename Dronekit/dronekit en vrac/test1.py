import multiprocessing
import time
from Utils import *

def fn1(connexion):

    while connexion.recv() != None:
        print("connexion Ok")
        print(connexion.recv())



def main(): 

    connexionA, connexionB = multiprocessing.Pipe()

    proc1 = multiprocessing.Process(target=RunDetection, args=(connexionA,))
    proc2 = multiprocessing.Process(target=fn1, args=(connexionB,))
    proc1.start()
    proc2.start()
    proc1.join()
    proc2.join()

if __name__ == "__main__":
        main()