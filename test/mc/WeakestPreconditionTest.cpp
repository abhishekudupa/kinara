#include <iostream>
#include <string>

#include "ElevatorLTS.hpp"
#include "../../src/mc/LTSChecker.hpp"
#include "../../src/mc/StateVecPrinter.hpp"
#include "../../src/mc/Trace.hpp"

using namespace std;

using namespace ESMC;
using namespace MC;


ExpT WeakestPrecondition(ExpT Phi, TraceBase* Trace);


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


ExpT WeakestPrecondition(ExpT InitialPhi, TraceBase* Trace) {
    ExpT Phi = InitialPhi;
    const vector<TraceElemT>& TraceElements = Trace->As<SafetyViolation>()->GetTraceElems();
    for (auto it = TraceElements.rbegin(); it != TraceElements.rend(); ++it) {
        GCmdRef guarded_command = it->first;
        const vector<LTSAssignRef>& updates = guarded_command->GetUpdates();
        const ExpT& guard = guarded_command->GetGuard();

        MgrT::SubstMapT SubstMapForTransMsg;
        for (LTSAssignRef update: updates) {
            const ExpT& lhs = update->GetLHS();
            if (lhs->Is<OpExpression>()) {
                auto lhs_as_op = lhs->As<OpExpression>();
                auto children = lhs_as_op->GetChildren();
                auto lhs_base = children[0];
                if (lhs_base->Is<VarExpression>()) {
                    auto lhs_base_var = lhs_base->As<VarExpression>();
                    if (lhs_base_var->GetVarName() == "__trans_msg__") {
                        SubstMapForTransMsg[lhs] = update->GetRHS();
                    }
                }
            }
        }
        MgrT::SubstMapT SubstMapForTransition;
        for (LTSAssignRef update: updates) {
            auto lhs = update->GetLHS();
            auto rhs = update->GetRHS();
            SubstMapForTransition[lhs] = rhs->GetMgr()->SubstituteForWP(SubstMapForTransMsg, rhs);
        }

        Phi = Phi->GetMgr()->SubstituteForWP(SubstMapForTransition, Phi);
        Phi = Phi->GetMgr()->MakeExpr(LTSOps::OpIMPLIES, guard, Phi);
    }
    return Phi;
}
