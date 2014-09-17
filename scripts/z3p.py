import z3

def Eq(arg1, arg2):
    if arg2 is None:
        return False
    a, b = z3._coerce_exprs(arg1, arg2)
    return z3.BoolRef(z3.Z3_mk_eq(arg1.ctx_ref(), a.as_ast(), b.as_ast()), arg1.ctx)

z3.__dict__['Eq'] = Eq

def Neq(arg1, arg2):
    return z3.Not(z3.Eq(arg1, arg2))

z3.__dict__['Neq'] = Neq

def is_bool_or_int_value(value):
    if z3.is_int_value(value):
        return True
    if value.sort() == z3.BoolSort():
        if z3.is_true(value) or z3.is_false(value):
            return True
    else:
        return False

z3.__dict__['is_bool_or_int_value'] = is_bool_or_int_value

def structural_equality(self, other):
    if other is None:
        return False
    return self.eq(other)

z3.ExprRef.__eq__ = structural_equality

z3.FuncDeclRef.__eq__ = structural_equality

z3.ExprRef.__ne__ = lambda self, other: not structural_equality(self, other)

z3.AstRef.__hash__ = z3.AstRef.hash

z3.AstRef.__deepcopy__ = lambda self, memo: self

def variables_in_expression(expression):
    expressions = [expression]
    variables = []
    while len(expressions) > 0:
        expression = expressions.pop(0)
        if z3.is_const(expression):
            if not z3.is_bool_or_int_value(expression):
                variables.append(expression)
        elif z3.is_app(expression):
            if expression.decl().kind() == z3.Z3_OP_UNINTERPRETED:
                variables.append(expression.decl())
            for i in range(expression.num_args()):
                expressions.append(expression.arg(i))
    return variables

z3.__dict__['variables_in_expression'] = variables_in_expression



from z3 import *
