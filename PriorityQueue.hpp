
#pragma once

#include <vector>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <functional>


template <class E>
class PriorityQueue
{
    private:
        struct Node
        {
            E elem;
            double key;
        };

        const double MAX = std::numeric_limits<double>::infinity();

        std::vector<Node> _elems;
        std::unordered_map<E,int,std::function<size_t(E)>,std::function<bool(E,E)> > _indexLookUp;
        int _size;

        void buildMinHeap(void)
        {
            int startIndex = _size/2;

            for(int i = startIndex; i >= 0; i--)
            {
                minHeapify(i);
            }
        }
        void swap(int ind1,int ind2)
        {
            Node tmp = _elems[ind1];

            _elems[ind1] = _elems[ind2];
            _elems[ind2] = tmp;

            _indexLookUp[_elems[ind1].elem] = ind1;
            _indexLookUp[_elems[ind2].elem] = ind2;
        }
        void minHeapify(int index)
        {
            int indexOfMin = 0;
            int left = leftChild(index);
            int right = rightChild(index);

            if(left >= 0 && _elems[index].key > _elems[left].key)
            {
                indexOfMin = left;
            }
            else
            {
                indexOfMin = index;
            }

            if(right >= 0 && _elems[indexOfMin].key > _elems[right].key)
            {
                indexOfMin = right;
            }

            if(indexOfMin != index)
            {
                swap(indexOfMin,index);

                minHeapify(indexOfMin);
            }
        }
        int leftChild(int index)
        {
            int res = 2*index + 1;

            return (res < _size) ? res : -1;
        }
        int rightChild(int index)
        {
            int res = 2*index + 2;

            return (res < _size) ? res : -1;
        }
        int parent(int index)
        {
            return (index <= 0) ? -1 : ((index - 1)/2);
        }

    public:
        PriorityQueue(std::function<size_t(E)> hashFunctor, std::function<bool(E,E)> equalityFunctor)
        {

            std::unordered_map<E,int,std::function<size_t(E)>,std::function<bool(E,E)> > m(0,hashFunctor,equalityFunctor);

            _indexLookUp = m;
            
            _size = 0;
        }

        bool isEmpty()
        {
            return (_size == 0);
        }

        E peek(void)
        {
            return _elems[0].elem;
        }
        E poll(void)
        {
            E result = _elems[0].elem;

            _elems[0] = _elems.back();
            auto it = _indexLookUp.find(_elems.back().elem);
            _indexLookUp.erase(it);
            _elems.pop_back();

            _indexLookUp[_elems[0].elem] = 0;


            _size -= 1; 

            minHeapify(0);

            return result;
        }
        void decreaseKey(E elem, double newKey)
        {
            int index = _indexLookUp[elem];

            if(_elems[index].key > newKey)
            {
                _elems[index].key = newKey;

                int current = index;
                while(current > 0)
                {
                    if(_elems[current].key < _elems[parent(current)].key)
                    {
                        swap(current,parent(current));

                        current = parent(current);
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
        void insert(E e, double k)
        {
            Node n;
            n.elem = e;
            n.key = MAX;

            _elems.push_back(n);

            _indexLookUp[e] = _size;

            _size += 1;

            decreaseKey(e,k);
        }
        void update(E e, double k)
        {
            if(_indexLookUp.find(e) != _indexLookUp.end())
            {
                decreaseKey(e,k);
            }
            else
            {
                insert(e,k);
            }
        }
        void merge(PriorityQueue<E> other)
        {
            for(unsigned long i = 0; i < other._elems.size(); i++)
            {
                _elems.push_back(other._elems[i]);
                _indexLookUp[_elems.back().elem] = _size + i;
                _size += 1;
            }

            buildMinHeap();
        }

        void print(void)
        {
            if(_elems.size() == 0)
            {
                std::cout << "empty queue" << std::endl;
            }

            for(auto node : _elems)
            {
                std::cout << "key: " << node.key << ", ";
            }

            std::cout << std::endl;
        }

};

