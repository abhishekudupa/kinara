import os.path
import sys
import tempfile

import z3p


UNIT_SORT = z3p.DeclareSort('Unit')


def absolute_filename_in_tmp(filename):
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '../tmp/{}'.format(filename)))


def add_indent(code, indent_string, add_to_first_line=True):
    if add_to_first_line:
        temp = indent_string + code.replace('\n', '\n' + indent_string)
    else:
        temp = code.replace('\n', '\n' + indent_string)
    temp = temp.rstrip(' ')
    return temp


def file_in_temp(filename=None, **kwargs):
    temp_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../tmp/'))
    if filename is not None:
        return open(os.path.join(temp_dir, filename), 'w')
    else:
        return tempfile.NamedTemporaryFile(dir=temp_dir,
                                           delete=False,
                                           **kwargs)


def get_variable_from_module(module_path, variable_name):
    module = import_module(module_path)
    if module is None:
        sys.stderr.write('File {} does not exist.\n'.format(module_path))
        return None
    try:
        return getattr(module, variable_name)
    except AttributeError:
        sys.stderr.write('Variable {} was not found in {}\n'.format(variable_name, module_path))

def model_filename(model_name):
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '{}.py'.format(model_name)))


def new_variable(name, var_type):
    if var_type == 'int':
        var = z3p.Int(name)
        var.name = name
        return var
    elif var_type == 'unit':
        var = z3p.Const(name, UNIT_SORT)
        var.name = name
        return var
    else:
        raise NotImplementedError('Cannot create variables of type {}'.format(var_type))


def import_module(module_path):
    if not os.path.isfile(module_path):
        return None
    module_dir = os.path.dirname(module_path)
    module_filename = os.path.basename(module_path)
    module_name = os.path.splitext(module_filename)[0]
    sys.path.insert(0, module_dir)
    import importlib
    module = importlib.import_module(module_name)
    return module


def negate_z3_constraint(constraint):
    if constraint.decl().kind() == z3p.Z3_OP_NOT:
        return constraint.arg(0)
    else:
        return z3p.Not(constraint)


def python_value_to_z3(value):
    if isinstance(value, bool):
        return z3p.BoolVal(value)
    elif isinstance(value, int) or isinstance(value, long):
        return z3p.IntVal(value)
    else:
        raise NotImplementedError("unknown value {}".format(value))


def string_to_python_value(value):
    return int(value)


def z3_to_python_value(value):
    """
    is_expr: Return `True` if `a` is a Z3 expression, is_expr(1) is False
    """
    assert z3p.is_expr(value)
    if z3p.is_int_value(value):
        return value.as_long()
    elif value.sort() == z3p.BoolSort():
        if z3p.is_true(value):
            return True
        elif z3p.is_false(value):
            return False
    raise NotImplementedError


def z3_function_type(function):
    assert z3p.is_func_decl(function)
    return ([function.domain(i) for i in range(function.arity())] +
            [function.range()])


def z3_expression_to_nusmv(expression, tables=None, other_tables=None):
    z3_to_nusmv = {z3p.Z3_OP_AND: ' & ',
                   z3p.Z3_OP_ADD: ' + ',
                   z3p.Z3_OP_MOD: ' mod ',
                   z3p.Z3_OP_EQ: ' = ',
                   z3p.Z3_OP_LT: ' < ',
                   z3p.Z3_OP_GT: ' > '}
    kind = expression.decl().kind()
    if (z3p.is_const(expression) and
            expression.decl().name().startswith('table_')):
        assert tables is not None
        table_name = expression.decl().name()
        assert table_name in tables or table_name in other_tables
        table_expression = tables[table_name] if table_name in tables else other_tables[table_name]
        case_expression = 'case '
        for condition, result in table_expression:
            c = z3_expression_to_nusmv(condition, tables)
            r = z3_expression_to_nusmv(result, tables)
            case_expression += '{} : {}; '.format(c, r)
        case_expression += 'esac'
        return case_expression
    elif kind in z3_to_nusmv:
        children = [z3_expression_to_nusmv(c, tables) for c in expression.children()]
        return '({})'.format(z3_to_nusmv[kind].join(children))
    elif expression == z3p.BoolVal(True):
        return 'TRUE'
    elif expression == z3p.BoolVal(False):
        return 'FALSE'
    elif kind == z3p.Z3_OP_NOT:
        e = z3_expression_to_nusmv(expression.arg(0))
        return '!({})'.format(e)
    elif z3p.is_const(expression):
        return str(expression)
    elif kind == z3p.Z3_OP_ITE:
        guard = z3_expression_to_nusmv(expression.arg(0))
        then_branch = z3_expression_to_nusmv(expression.arg(1))
        else_branch = z3_expression_to_nusmv(expression.arg(2))
        return '(case {} : {}; TRUE : {}; esac)'.format(guard, then_branch, else_branch)
    else:
        raise NotImplementedError('Cannot translate expression {} to nusmv.'.format(expression))


def selects_in_expression(expression):
    expressions = [expression]
    selects = []
    while len(expressions) > 0:
        expression = expressions.pop(0)
        if z3p.is_const(expression):
            continue
        elif z3p.is_app(expression):
            if expression.decl().kind() == z3p.Z3_OP_SELECT:
                selects.append(expression)
            else:
                for i in range(expression.num_args()):
                    expressions.append(expression.arg(i))
    return selects


def evaluate_expression(expression, symbolic_memory, concrete_memory, tables=None):
    if isinstance(expression, int):
        return expression
    if isinstance(expression, tuple):
        return expression
    if z3p.is_const(expression):
        if z3p.is_bool_or_int_value(expression):
            return expression
        if expression in symbolic_memory:
            return symbolic_memory[expression]
        elif expression.decl().name().startswith('table_'):
            assert expression.decl().name() in tables
            table_expression = tables[expression.decl().name()]
            first_value = table_expression[0][1]
            if first_value.sort() == z3p.IntSort():
                last_else = z3p.IntVal(0)
            else:
                last_else = z3p.BoolVal(False)
            if_expression = None
            for conditional, value in table_expression:
                guard = evaluate_expression(conditional, symbolic_memory, concrete_memory)
                last_else = z3p.If(guard, value, last_else)
            return z3p.simplify(last_else)
        else:
            return concrete_memory[expression]
    elif expression.decl().kind() == z3p.Z3_OP_SELECT:
        if expression in symbolic_memory:
            return symbolic_memory[expression]
        else:
            return concrete_memory[expression]
    else:
        new_args = [evaluate_expression(expression.arg(i), symbolic_memory, concrete_memory, tables)
                    for i in range(expression.num_args())]
        if expression.decl().kind() == z3p.Z3_OP_AND:
            return z3p.And(*new_args)
        else:
            return expression.decl()(*new_args)
