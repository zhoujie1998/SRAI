#include "ros/ros.h"
void A(int a)
{
    int i;

    if (a>6) return;
    i=0;
    A(i);
}
int main(int argc, char **argv)
{
    int a=1;
    A(a);
    return 0;
}