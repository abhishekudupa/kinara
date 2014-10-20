#include <iostream>

#include "../../src/common/FwdDecls.hpp"
#include "../../src/uflts/LTSTypes.hpp"
#include "../../src/uflts/LTSUtils.hpp"
#include "../../src/symexec/LTSAnalyses.hpp"

using namespace std;

using namespace ESMC;
using namespace LTS;
using namespace Analyses;


int main() {
    auto mgr = new MgrT();
    auto zero_to_ten_range_type = mgr->MakeType<ExprRangeType>(0, 10);
    auto array_type = mgr->MakeType<ExprArrayType>(zero_to_ten_range_type,
                                                   zero_to_ten_range_type);
    vector<pair<string, ExprTypeRef>> record_members;
    record_members.push_back(make_pair("array_field", array_type));
    auto record_type = mgr->MakeType<ExprRecordType>("RecordType", record_members);
    auto record_variable = mgr->MakeVar("record_variable", record_type);
    auto array_index = mgr->MakeVal("1", zero_to_ten_range_type);
    auto FAType = mgr->MakeType<ExprFieldAccessType>();
    auto array_field_exp = mgr->MakeExpr(LTSOps::OpField,
                                         record_variable,
                                         mgr->MakeVar("array_field", FAType));
    auto index_value = mgr->MakeVar("i", zero_to_ten_range_type);
    auto array_expression = mgr->MakeExpr(LTSOps::OpIndex, array_field_exp, index_value);

    auto array_expression_in_path = mgr->MakeExpr(LTSOps::OpIndex, array_field_exp, mgr->MakeVar("j", zero_to_ten_range_type));
    MgrT::SubstMapT substitution_map;
    substitution_map[array_expression] = mgr->MakeVal("1", zero_to_ten_range_type);
    cout << "Substitution map is: " << endl;
    for (auto Pair: substitution_map) {
        cout << Pair.first->ToString() << " = " << Pair.second->ToString() << endl;
    }
    auto path_condition = mgr->MakeExpr(LTSOps::OpGT, array_expression_in_path, mgr->MakeVal("0", zero_to_ten_range_type));
    cout << "Path condition before substitution is " << path_condition->ToString() << endl;
    auto new_path_condition = mgr->ApplyTransform<SubstitutorForWP>(path_condition, substitution_map);
    cout << "Path condition after substitution is " << new_path_condition->ToString() << endl;

    // multiple array assignments
    auto index_value_k = mgr->MakeVar("k", zero_to_ten_range_type);
    auto array_expression_k = mgr->MakeExpr(LTSOps::OpIndex, array_field_exp, index_value_k);
    substitution_map[array_expression_k] = mgr->MakeVal("5", zero_to_ten_range_type);
    cout << "Substitution map is: " << endl;
    for (auto Pair: substitution_map) {
        cout << Pair.first->ToString() << " = " << Pair.second->ToString() << endl;
    }
    new_path_condition = mgr->ApplyTransform<SubstitutorForWP>(path_condition, substitution_map);
    cout << "Path condition after substitution is " << new_path_condition->ToString() << endl;
}
