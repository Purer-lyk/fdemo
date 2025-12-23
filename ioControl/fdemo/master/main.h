#ifndef MAIN_H
#define MAIN_H

#include <cstdio>
#include <csignal>
#include <atomic>

extern int main();

extern std::atomic<bool> run_;
extern void signalHandler(int);


#endif
