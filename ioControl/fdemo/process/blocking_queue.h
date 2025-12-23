#ifndef BLOCKING_QUEUE_H
#define BLOCKING_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

template<typename T> 
class BlockinQueue{
private:
	std::queue queue_;
	std::mutex mtx_;
	std::condition_variable cv_;
	bool stop_;
	int maxsize;
	
public:
	BlockinQueue():stop_(false){}
	
	void push(const T& item){
		{
			std::lock_gurad<std::mutex> lock(mtx_);
			queue_.push(item);
		}
		cv_.notify_one();
	}
	
	void pop(const T& item){
		{
			std::unique_lock<std::mutex> lock(mtx_);
			
		}
	}
	
	void stop(){
		{
			std::lock_gurad<std::mutex> lock(mtx_);
			stop_=true;
		}
		cv_.notify_all();
	}
};

#endif
