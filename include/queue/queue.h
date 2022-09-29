#ifndef QUEUE_H
#define QUEUE_H

#include <queue>
#include <mutex>

template <class T>
class queue {
private:
	std::queue<T> *readQueue = new std::queue<T>();
	std::queue<T> *writeQueue = new std::queue<T>();

	std::mutex writeMtx;

	int readElems = 0;

public:
	bool pop(T &ret) {
		if (readElems > 0) {
			readElems--;
			ret = readQueue->front();
			readQueue->pop();
			return true;
		}
		std::unique_lock lock(writeMtx);
		if (writeQueue->size() == 0)
			return false;
		auto tmp = readQueue;
		readQueue = writeQueue;
		writeQueue = tmp;
		readElems = readQueue->size();
		readElems--;
		ret = readQueue->front();
		readQueue->pop();
		return true;
	}

	bool push(T const &val) {
		std::unique_lock lock(writeMtx);
		writeQueue->push(val);
		return true;
	}
};

#endif //QUEUE_H
