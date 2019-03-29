import argparse 
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig,ax = plt.subplots()
args = None

def getdata(args):

    f = open(args.log_file)
    
    lines  = [line.rstrip("\n") for line in f.readlines()]
    
    numbers = {'1','2','3','4','5','6','7','8','9'}

    iters = []
    loss = []
    
    for line in lines:
        args = line.split(' ')
        if args[0][-1:]==':' and args[0][0] in numbers :
            if iters==[]:
                iters.append(1)            
                loss.append(float(args[2]))
            else:
                iters.append(iters[-1]+1)            
                loss.append(float(args[2]))

    return iters, loss

def animate(self):

    X,Y = getdata(args)

    Lavg = sum(Y[-100:])/100
    ax.clear()
#    ax.set_xlim(max(0, X[-1]-100),X[-1]+50)
#    ax.set_ylim(Lavg*0.8,Lavg*1.2)   
    ax.plot(X,Y)

def main(argv):

    parser = argparse.ArgumentParser()


    parser.add_argument(
        "log_file",
        help = "path to log file"
        )
    global args
    args = parser.parse_args()

    plt.xlabel('iterations')
    plt.ylabel('loss')
    plt.grid()
    
    ani = animation.FuncAnimation(fig, animate, interval = 10000)

    plt.show()
    
if __name__ == "__main__":
    main(sys.argv)
