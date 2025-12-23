#include "main.h"
#include "main_system.h"

std::atomic<bool> run_(true);

void signalHandler(int){
    run_ = false;
}

int main(){
	mainSystem worker;
	signal(SIGINT, signalHandler);
	worker.run();
	return 0;
}
