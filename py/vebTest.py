
from vanemdeboas import VEBQueueBase

def main():

    q = VEBQueueBase(2**32)

    for i in range(0,100000):
        q.insert(i)


    for i in range(0,100000):
        m = q.min()
        q.delete(m)





if __name__ == "__main__":
    main()
