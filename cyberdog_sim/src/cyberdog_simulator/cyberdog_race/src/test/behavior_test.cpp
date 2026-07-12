#include "behavior_test.hpp"

namespace behavior_test {

void ping() { fprintf(stderr, "\033[1;36m[Test] pong\033[0m\n"); }
void run_all() {
    fprintf(stderr, "\033[1;36m[Test] run_all ---\033[0m\n");
    ping();
    fprintf(stderr, "\033[1;36m[Test] run_all done\033[0m\n");
}

}
