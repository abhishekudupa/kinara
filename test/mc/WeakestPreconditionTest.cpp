#include <iostream>
#include <string>

#include "ElevatorLTS.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/StateVecPrinter.hpp"
#include "../../src/mc/Trace.hpp"
<<<<<<< HEAD
#include "../../src/symexec/LTSAnalyses.hpp"
=======
>>>>>>> 6b20eee4104abfec37398ea174a457db6a18b245

using namespace std;

using namespace ESMC;
using namespace MC;
using namespace Analyses;


int main() {
    auto the_lts = ElevatorLTS();
    auto checker = new LTSChecker(the_lts);
    auto traces = checker->BuildAQS();
    SafetyViolation* safety_trace;
    for (auto trace: traces) {
        if (!trace->Is<DeadlockViolation>() && trace->Is<SafetyViolation>()) {
            safety_trace = trace->As<SafetyViolation>();
        }
    }
    ExpT phi = the_lts->MakeTrue();
    cout << WeakestPrecondition(phi, safety_trace)->ToString() << endl;
}
