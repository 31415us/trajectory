
from heapdict import heapdict

def main():
    q = heapdict()
    for i in range(0,100000):
        q[i] = i

    for i in range(0,100000):
        q.popitem()


if __name__ == "__main__":
    main()
