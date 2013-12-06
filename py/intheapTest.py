
from IntegerHeap import IntegerHeap

def main():
    q = IntegerHeap(5)

    for i in range(0,100000):
        q.add(i)

    for i in range(0,100000):
        m = q.min()
        q.remove(m)
    

if __name__ == "__main__":
    main()
