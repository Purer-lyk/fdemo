#include "main.h"
#include "thread_system.h"

using namespace std;

std::atomic<bool> run_(true);

void signalHandler(int){
    run_ = false;
}

int main(){
    threadSystem worker;
    signal(SIGINT, signalHandler);
    worker.start();
    worker.join();
    return 0;
}
