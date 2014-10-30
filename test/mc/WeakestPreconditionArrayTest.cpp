#include <iostream>

#include "../../src/common/FwdDecls.hpp"
#include "../../src/uflts/LTSTypes.hpp"
#include "../../src/uflts/LTSUtils.hpp"
#include "../../src/symexec/LTSAnalyses.hpp"

using namespace std;

using namespace ESMC;
using namespace LTS;
using namespace Analyses;


bool AreExpressionsUnifiable(ExpT e1, ExpT e2, vector<ExpT>& Conditions) {
    if (e1->Is<OpExpression>() && e2->Is<OpExpression>()) {
        auto e1AsOp = e1->As<OpExpression>();
        auto e2AsOp = e2->As<OpExpression>();
        auto e1OpCode = e1AsOp->GetOpCode();
        auto e2OpCode = e2AsOp->GetOpCode();
        if (e1OpCode == LTSOps::OpField && e2OpCode == LTSOps::OpField) {
            auto e1Field = e1AsOp->GetChildren()[1];
            auto e2Field = e2AsOp->GetChildren()[1];
            if (e1Field == e2Field) {
                auto e1Base = e1AsOp->GetChildren()[0];
                auto e2Base = e2AsOp->GetChildren()[0];
                return AreExpressionsUnifiable(e1Base, e2Base, Conditions);
            } else {
                return false;
            }
        } else if (e1OpCode == LTSOps::OpIndex && e2OpCode == LTSOps::OpIndex) {
            auto e1Index = e1AsOp->GetChildren()[1];
            auto e2Index = e2AsOp->GetChildren()[1];
            auto e1Base = e1AsOp->GetChildren()[0];
            auto e2Base = e2AsOp->GetChildren()[0];
            Conditions.push_back(e1->GetMgr()->MakeExpr(LTSOps::OpEQ, e1Index, e2Index));
            return AreExpressionsUnifiable(e1Base, e2Base, Conditions);
        } else {
            return false;
        }
    }
    return (e1 == e2);
}


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


    // a[1].Data

    vector<pair<string, ExprTypeRef>> record_members_a;
    record_members_a.push_back(make_pair("Data", zero_to_ten_range_type));
    auto record_type_a = mgr->MakeType<ExprRecordType>("RecordTypeA", record_members_a);
    auto array_type_a = mgr->MakeType<ExprArrayType>(zero_to_ten_range_type,
                                                     record_type_a);

    auto index_value_i = mgr->MakeVar("i", zero_to_ten_range_type);

    auto index_value_1 = mgr->MakeVar("1", zero_to_ten_range_type);

    auto array_var_a = mgr->MakeVar("a", array_type_a);

    auto array_expression_a_i = mgr->MakeExpr(LTSOps::OpIndex, array_var_a, index_value_i);

    auto array_expression_a_1 = mgr->MakeExpr(LTSOps::OpIndex, array_var_a, index_value_1);

    auto array_field_exp_a_i_data = mgr->MakeExpr(LTSOps::OpField,
                                                  array_expression_a_i,
                                                  mgr->MakeVar("Data", FAType));

    auto array_field_exp_a_1_data = mgr->MakeExpr(LTSOps::OpField,
                                                  array_expression_a_1,
                                                  mgr->MakeVar("Data", FAType));

    cout << array_field_exp_a_i_data << endl;
    cout << array_field_exp_a_1_data << endl;

    vector<ExpT> Conditions;
    auto value = AreExpressionsUnifiable(array_field_exp_a_i_data, array_field_exp_a_1_data, Conditions);
    cout << value << endl;
    for (auto Exp : Conditions) {
        cout << Exp << endl;
    }

    vector<pair<string, ExprTypeRef>> b_index_record_members;
    b_index_record_members.push_back(make_pair("a", array_type_a));
    auto record_type_b = mgr->MakeType<ExprRecordType>("RecordTypeB", b_index_record_members);

    auto array_type_b = mgr->MakeType<ExprArrayType>(zero_to_ten_range_type,
                                                     record_type_b);

    auto index_value_j = mgr->MakeVar("j", zero_to_ten_range_type);

    auto index_value_3 = mgr->MakeVar("3", zero_to_ten_range_type);

    auto array_var_b = mgr->MakeVar("b", array_type_b);

    auto b_j = mgr->MakeExpr(LTSOps::OpIndex, array_var_b, index_value_j);

    auto b_1 = mgr->MakeExpr(LTSOps::OpIndex, array_var_b, index_value_1);

    auto b_j_a = mgr->MakeExpr(LTSOps::OpField,
                               b_j,
                               mgr->MakeVar("a", FAType));

    auto b_1_a = mgr->MakeExpr(LTSOps::OpField,
                               b_1,
                               mgr->MakeVar("a", FAType));

    auto b_j_a_3 = mgr->MakeExpr(LTSOps::OpIndex,
                                 b_j_a,
                                 index_value_3);

    auto b_1_a_i = mgr->MakeExpr(LTSOps::OpIndex,
                                 b_1_a,
                                 index_value_i);

    auto b_j_a_3_data = mgr->MakeExpr(LTSOps::OpField,
                                      b_j_a_3,
                                      mgr->MakeVar("Data", FAType));

    auto b_1_a_i_data = mgr->MakeExpr(LTSOps::OpField,
                                      b_1_a_i,
                                      mgr->MakeVar("Data", FAType));

    cout << b_j_a_3_data << endl;
    cout << b_1_a_i_data << endl;

    Conditions.clear();
    value = AreExpressionsUnifiable(b_j_a_3, b_1_a_i, Conditions);
    cout << value << endl;
    for (auto Exp : Conditions) {
        cout << Exp << endl;
    }

}
