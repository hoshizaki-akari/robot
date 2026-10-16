// Test-only dynamic interposer. Abort before ANY real vendor SDK construction.
// The negative-control test proves interposition actually works.
#include <unistd.h>
extern "C" void _ZN16ServiceInterfaceC1Ev(void *) {_exit(91);}
extern "C" void _ZN16ServiceInterfaceC2Ev(void *) {_exit(91);}
