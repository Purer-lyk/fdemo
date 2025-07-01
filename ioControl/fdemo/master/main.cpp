#include "main.h"
#include "main_system.h"

#define MODEL "/home/l/Pack/model/ppyolo_5.nb"

int main(){
	mainSystem worker(MODEL);
	worker.run();
	return 0;
}
