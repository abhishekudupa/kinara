import itertools
import z3p


class ConstraintModel(object):
    def add_function(self, function_name,
                     domain_types, range_type,
                     domain_values, range_constraint=None, function_defined=False):
        """Add a function in the model.

        The domain and range types are z3 sorts; the domain types is a
        list of domain types.  Domain values is a list of lists of
        python values of the corresponding types.  Range constraint is
        given as a function of two arguments, the first representing
        the function and the second the argument, e.g., lambda f, i:
        f(i) > 1.

        """
        signature = domain_types + [range_type]
        if isinstance(function_name, str):
            signature = domain_types + [range_type]
            f = z3p.Function(function_name, *signature)
        else:
            f = function_name
            function_name = f.name()
        self.function_names.append(function_name)
        self.function_name_to_domain_values[function_name] = []
        self.z3_functions_by_name[function_name] = f
        for arguments in itertools.product(*domain_values):
            if range_constraint is not None and hasattr(range_constraint, '__call__'):
                if not function_defined:
                    self.solver.add(range_constraint(*([f] + list(arguments))))
                else:
                    self.solver.add(range_constraint(*arguments))
            self.function_name_to_domain_values[function_name].append(arguments)
        if range_constraint is not None and not hasattr(range_constraint, '__call__'):
            self.solver.add(range_constraint)

    def add_constraint(self, constraint):
        """Add specific z3 to the solver.
        """
        self.constraints.append(constraint)
        self.solver.add(constraint)

    def get_z3_function(self, function_name):
        return self.z3_functions_by_name[function_name]

    def __init__(self):
        """TODO
        """
        self.function_names = []
        self.function_name_to_domain_values = {}
        self.z3_functions_by_name = {}
        self.solver = z3p.Solver()
        self.solver.set(unsat_core=True)
        self.constraints = []

    def inputs_in_constraints_for_function(self, function_name, table):
        retval = []
        for constraint in self.constraints:
            expressions = [constraint]
            while len(expressions) > 0:
                expression = expressions.pop()
                if not z3p.is_const(expression):
                    if expression.decl().name() == function_name:
                        retval.append(expression)
                    else:
                        for i in range(expression.num_args()):
                            expressions.append(expression.arg(i))
        return retval

    def lookup_table_for_function(self, function_name):
        """Returns a model for the function as a dictionary.
        The dictionary has tuples of argument values as keys. In particular,
        if the function has a single argument, the key will still be a single element tuple.
        """
        assert (function_name in self.function_names and
                function_name in self.z3_functions_by_name)
        z3_function = self.z3_functions_by_name[function_name]
        model = self.solver.model()
        table = {}
        for i in self.function_name_to_domain_values[function_name]:
            table[i] = model.evaluate(z3_function(i))
            # If a value is unconstrained the z3 model might just return
            # a variable back, therefore for the boolean case
            if not z3p.is_bool_or_int_value(table[i]) and table[i].sort() == z3p.BoolSort():
                # a constant value is returned here.
                table[i] = z3p.BoolVal(False)
        return table

    def conditional_expression_for_function(self, function_name, input_variables):
        """Translate look-up table to automaton conditional expression.
        Given input variables as z3 variables, translate look-up table
        to automaton conditional expression.
        If the function is one with no inputs, i.e., a constant,
        that constant value is returned.
        """
        return lookup_table_to_conditional_expression(self.lookup_table_for_function(function_name),
                                                      input_variables)

    def lookup_table_to_negative_constraint(self, function_name):
        """Translate look-up table to negative constraint.
        Given input variables as z3 variables, translate look-up table
        to a constraint that guarantees that the same look-up table will not
        be a solution.
        """
        lookup_table = self.lookup_table_for_function(function_name)
        constraint = lambda f: z3p.Or([z3p.Neq(f(*input_values), output_value)
                                       for input_values, output_value in lookup_table.items()])
        return constraint

    def lookup_table_for_inputs_to_negative_constraint(self, function_name, input_values_set):
        """Translate look-up table to negative constraint for specific inputs.
        """
        lookup_table = self.lookup_table_for_function(function_name)
        constraint = lambda f: z3p.Or([z3p.Neq(f(*input_values), output_value)
                                       for input_values, output_value in lookup_table.items()
                                       if set(input_values) in input_values_set])
        return constraint

    def solve(self):
        """Simple call check on z3
        """
        self.solver.check()


def lookup_table_to_conditional_expression(lookup_table, input_variables):
    conditional_expression = []
    for input_values, output_value in lookup_table.items():
        if len(input_values) == 1:
            conditional = z3p.Eq(input_variables[0], input_values[0])
        elif len(input_values) == 0:
            assert len(lookup_table.values()) == 1
            return [(z3p.BoolVal(True), output_value)]
        else:
            conditional = z3p.And([z3p.Eq(var, val) for var, val in zip(input_variables, input_values)])
        conditional_expression += [(conditional, output_value)]
    return conditional_expression
