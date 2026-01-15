#include "main.h"
#include "main_system.h"
#include "hik_capture.h"

std::atomic<bool> run_(true);

void signalHandler(int){
    signal(SIGTERM, SIG_IGN);
    run_ = false;
    hik_in = 0;
    hik_run = 0;
}

int main(){
	mainSystem worker;
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);
	worker.run();
	return 0;
}
