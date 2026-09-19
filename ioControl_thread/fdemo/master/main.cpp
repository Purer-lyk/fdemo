#include "main.h"
#include "main_system.h"
#include "sample.h"

std::atomic<bool> run_(true);

void signalHandler(int){
    signal(SIGTERM, SIG_IGN);
    run_ = false;
    isRUNNING = false;
}

int main(int argc, char* argv[]){
    if (argc < 2)
    {
        printf("usage: %s <config_file>\n", argv[0]);
        return -1;
    }
    config config_obj;
    if (config_obj.parse_config(argv[1]) != 0)
    {
        printf("parse config failed\n");
        return -1;
    }
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    mainSystem* worker = new mainSystem();
    int ret = processMain(config_obj, worker);
    
    delete worker;
    worker=nullptr;
    
    return 0;
}
