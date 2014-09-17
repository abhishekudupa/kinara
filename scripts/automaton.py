"""
TODO Move the spin model out of here, make unit sort global again.
"""

import itertools
import networkx
import z3p
import util

import constraint_model


class SymbolicAutomaton(networkx.MultiDiGraph):
    """ This follows the definitions in the document.
    For internal transitions, the channel field is None.
    """

    def add_transition(self, transition):
        if len(transition) == 6:
            kind = 'input'
            channel_expression = []
            name, source, channel, guard, update, target = transition
        self.add_edge(source, target,
                      channel=channel,
                      channel_expression=channel_expression,
                      guard=guard,
                      update=update,
                      name=name,
                      kind=kind,
                      automaton=self)
        self.update_channel_edges()

    def alter_transition(self,
                         transition_name,
                         position,
                         new_expression):
        """Modify assignment expression of update in transition of automaton.
        position is either (assignment or update, index) or guard
        """
        transition = self.transition_by_name(transition_name)
        transition_attributes = transition[2]

        if isinstance(position, tuple):
            index = position[1]
            if position[0] == 'assignment':
                updates = transition_attributes['update']
                updates[index] = (updates[index][0], new_expression)
            elif position[0] == 'channel_expression':
                channel_expressions = transition_attributes['channel_expression']
                channel_expressions[index] = new_expression
        elif position == 'guard':
            transition_attributes['guard'] = new_expression
        else:
            raise NotImplementedError('Cannot replace expression in position {}'.format(position))

    def guard_enabled(self, state, guard, verbose=0):
        if guard.decl().name().startswith('table'):
            conditional = self.tables[guard.decl().name()]
            expression = z3p.BoolVal(False)
            for condition, result in conditional:
                expression = z3p.Or(expression, condition if z3p.is_true(result) else z3p.BoolVal(False))
            guard = expression
        guard_evaluated = util.evaluate_expression(guard, {}, state, self.tables)
        if verbose > 0:
            print 'Evaluated guard is {}'.format(guard_evaluated)
            print 'Simplified guard is {}'.format(z3p.simplify(guard_evaluated))
        s = z3p.Solver()
        s.add(z3p.Neq(guard_evaluated, True))
        if s.check() == z3p.unsat:
            return True
        else:
            return False

    def can_transition(self, state1, state2, transition):
        return state1['location'] == transition[0] and state2['location'] == transition[1] and self.guard_enabled(state1, transition[2]['guard'])

    def create_guard_hole(self, hole_name):
        """ variable is the name of the variable to update
        """
        signature = []
        domains = []
        for variable in self.variables:
            variable_sort = variable.sort()
            signature.append(variable_sort)
            if variable.sort() == z3p.IntSort():
                domains.append(range(self.variable_ranges[variable][0],
                                     self.variable_ranges[variable][1] + 1))
            elif variable.sort() == z3p.BoolSort():
                domains.append([z3p.BoolVal(False), z3p.BoolVal(True)])
            else:
                raise NotImplementedError("Unimplemented type of variable {}".format(variable_sort))
        signature.append(z3p.BoolSort())
        f = z3p.Function(hole_name, *signature)
        return (f, domains, None, None)

    def create_update_hole(self, variable_or_variable_name, hole_name):
        """ variable is the name of the variable to update
        """
        if isinstance(variable_or_variable_name, str):
            variable_to_update = next(v for v in self.variables
                                      if v.decl().name() == variable_or_variable_name)
        else:
            variable_to_update = variable_or_variable_name
        signature = []
        domains = []
        for variable in self.variables:
            variable_sort = variable.sort()
            signature.append(variable_sort)
            if variable.sort() == z3p.IntSort():
                domains.append([z3p.IntVal(i) for i in range(self.variable_ranges[variable][0],
                                                             self.variable_ranges[variable][1] + 1)])
            elif variable.sort() == z3p.BoolSort():
                domains.append([z3p.BoolVal(False), z3p.BoolVal(True)])
            else:
                raise NotImplementedError("Unimplemented type of variable {}".format(variable_sort))
        signature.append(variable_to_update.sort())
        f = z3p.Function(hole_name, *signature)
        constraint = z3p.And([z3p.Or([z3p.Eq(f(*arg), z3p.IntVal(result)) for result in range(self.variable_ranges[variable_to_update][0], self.variable_ranges[variable_to_update][1] + 1)]) for arg in itertools.product(*domains)])
        return (f, domains, constraint, (variable_to_update, self.variables.index(variable_to_update)))

    def draw(self, filename=None, transition_labels=False):
        """
        TODO: draw initial state
        """
        A = networkx.to_agraph(self)
        if filename is None:
            output = util.file_in_temp('temp.png')
        seen_edges = {}
        for source, target, attributes in self.edges(data=True):
            channel = attributes['channel']
            channel_expression = attributes['channel_expression']
            guard = attributes['guard']
            update = attributes['update']
            label = None
            if channel in self.input_channels:
                label = '{}?'.format(channel)
            elif channel in self.output_channels:
                if channel.sort() == util.UNIT_SORT:
                    label = '{}!'.format(channel)
                else:
                    label = '{}({})!'.format(channel, channel_expression)
            if not z3p.is_true(guard):
                if label:
                    label += '\n{}'.format(guard)
                else:
                    label = '{}'.format(guard)
            if update:
                if label:
                    label += '\n{}'.format(update)
                else:
                    label = '{}'.format(update)
            if transition_labels:
                label = attributes['name'] + ':' + label
            if (source, target) not in seen_edges:
                key = 0
                seen_edges[(source, target)] = 1
            else:
                key = seen_edges[(source, target)]
                seen_edges[(source, target)] += 1
            A.get_edge(source, target, key).attr['label'] = label
        A.layout(prog='dot')
        A.draw(output, format='png')
        output.close()

    def __eq__(self, other):
        if type(other) == str:
            return self.name == other
        return self.name == other.name

    def has_accept_location(self):
        return any(location == 'accept' for location in self.nodes())

    def has_error_location(self):
        return any(location == 'error' for location in self.nodes())

    def __hash__(self):
        return hash(self.name)

    def __init__(self, name, locations, initial_location, transitions, variables=None, initial_values=None, input_channels=None, output_channels=None, tables=None, variable_ranges=None, output_channel_ranges=None, compassion=None, justice=None):
        """ Every node represents a control location and has a unique name.
            Transitions are of the form (soruce, channel[, channel expression], guard, update, target)
            where source and target must be automaton control locations,
            channel an input or output channel,
            channel expression is there if the channel is an output one,
            guard is a z3 boolean expression,
            update is a pair (variable, expression).
        """
        super(SymbolicAutomaton, self).__init__()
        self.name = name
        self.variables = variables if variables is not None else []
        self.transitions = transitions
        self.initial_values = initial_values if initial_values is not None else []
        self.initials = {}
        for v, value in zip(self.variables, self.initial_values):
            self.initials[v] = value
        assert len(self.variables) == len(self.initial_values), "# of initial values should match # of variables"

        self.variable_initial_value = dict(zip([str(v) for v in self.variables],
                                               self.initial_values))
        # TODO change this to work with uninitialized variables
        # change variables input to include initial values
        self.input_channels = input_channels if input_channels is not None else []
        self.output_channels = output_channels if output_channels is not None else []
        # add channel with unit type unit can be an mtype with just one symbol

        for l in locations:
            self.add_node(l)

        for transition in transitions:
            channel_expression = []
            # if it is an input transition, there is no expression for the message to send
            if len(transition) == 6:
                name, source, channel, guard, update, target = transition
            # output transitions
            elif len(transition) == 7:
                name, source, channel, channel_expression, guard, update, target = transition
            # internal transitios
            elif len(transition) == 5:
                name, source, guard, update, target = transition
                channel = None
            assert source in locations and target in locations, transition
            assert (channel in self.input_channels or
                    channel in self.output_channels or
                    channel is None), transition
            if channel is not None:
                if len(channel_expression) == 0:
                    kind = 'input'
                else:
                    kind = 'output'
            else:
                kind = 'internal'
            if name is None:
                name = 't_{}'.format(self.number_of_edges)
            self.add_edge(source, target,
                          channel=channel,
                          channel_expression=channel_expression,
                          guard=guard,
                          update=update,
                          name=name,
                          kind=kind,
                          automaton=self)
        assert initial_location in self.nodes()
        self.initial_location = initial_location
        self.tables = {} if tables is None else tables

        self.channel_edges = {}
        self.update_channel_edges()
        self.changes = []
        self.transition_variables = {}
        self.update_transition_variables()
        self.variable_ranges = {} if variable_ranges is None else variable_ranges
        self.output_channel_ranges = {} if output_channel_ranges is None else output_channel_ranges
        for v in self.variables:
            if v.sort() == z3p.IntSort():
                assert v in self.variable_ranges, "No range declared for {}".format(v)
        for variable, variable_range in self.variable_ranges.items():
            assert variable in self.variables, "Variable {} was not declared".format(variable)
            assert variable.sort() == z3p.IntSort()
            assert isinstance(variable_range, tuple) and len(variable_range) == 2
            assert isinstance(variable_range[0], int)
            assert isinstance(variable_range[1], int)
            assert variable_range[0] < variable_range[1]
        for output_channel_or_field, output_channel_range in self.output_channel_ranges.items():
            if not z3p.is_const(output_channel_or_field) and output_channel_or_field.decl().kind() == z3p.Z3_OP_SELECT:
                assert output_channel_or_field.arg(0) in self.output_channels
            else:
                assert output_channel_or_field in self.output_channels
            # assert output_channel.sort() == z3p.IntSort()
            assert isinstance(output_channel_range, tuple) and len(output_channel_range) == 2
            assert isinstance(output_channel_range[0], int)
            assert isinstance(output_channel_range[1], int)
            assert output_channel_range[0] < output_channel_range[1]
        if compassion is not None:
            self.compassion = compassion
        else:
            self.compassion = []
        if justice is not None:
            self.justice = justice
        else:
            self.justice = []


    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return self.name

    def remove_transition_by_name(self, name):
        t = next(t for t in self.edges(data=True, keys=True) if t[3]['name'] == name)
        key = t[2]
        self.remove_edge(t[0], t[1], key=key)
        self.update_channel_edges()
        # t = next(t for t in self.transitions if t[2]['name'] == name)
        # self.transitions.remove(t)


    def revert_changes(self):
        """
        Revert edge attributes to the ones described in "changes".
        Changes is a list of triples, where a triple is of the form:
        (transition_name, transition_location, expression)
        where transition_name corresponds to the 'name' attribute of the edge,
        transition_location is 'channel_expression', 'guard', or 'update',
        and expression is the channel expression, guard, or rhs of update
        respectively.
        """
        for change in self.changes:
            t = self.transition_by_name(change[0])
            position = change[1]
            if isinstance(position, tuple):
                t[2][position[0]][position[1]] = change[2]
            else:
                t[2][position] = change[2]
    def substitute_hole(self, pairs):
        """
        Iterate over the edges of the automaton, substitute holes in every
        expression and record old values of expressions to be able to revert.
        """
        changes = []
        for edge in self.edges(data=True):
            data = edge[2]
            channel_expressions = data['channel_expression']
            for index, channel_expression in enumerate(channel_expressions):
                new_channel_expression = self.substitute_hole_in_expression(channel_expression, pairs)
                if not channel_expression == new_channel_expression:
                    changes.append((data['name'], ('channel_expression', index), channel_expression))
                    data['channel_expression'][index] = new_channel_expression
            guard = data['guard']
            new_guard = self.substitute_hole_in_expression(guard, pairs)
            if not guard == new_guard:
                changes.append((data['name'], 'guard', guard))
                data['guard'] = new_guard
            updates = data['update']
            for index, update in enumerate(updates):
                lhs, rhs = update
                new_rhs = self.substitute_hole_in_expression(rhs, pairs)
                if not rhs == new_rhs:
                    changes.append((data['name'], ('update', index), (lhs, rhs)))
                    data['update'][index] = (lhs, new_rhs)
        self.changes = changes
        return changes

    def substitute_hole_in_expression(self, expression, pairs):
        """
        Expression is a z3 expression, and pairs is a list of pairs
        of the form (function declaration representing hole, substitute)
        For now substitute can only be a value.
        TODO: change this so substitute can be a conditional expression as well.
        The latter will require to introduce a 'table variable'.
        """
        assert z3p.is_expr(expression)
        holes = [pair[0] for pair in pairs]
        if z3p.is_bool_or_int_value(expression):
            return expression
        if z3p.is_app(expression):
            declaration = expression.decl()
            for hole, substitute in pairs:
                if declaration == hole:
                    input_variables = [expression.arg(i)
                                       for i in range(expression.num_args())]
                    conditional = constraint_model.lookup_table_to_conditional_expression(substitute, input_variables)
                    # introduce table variable and add table declaration in tables
                    table_id = len(self.tables.keys())
                    table_var_name = 'table_{}'.format(table_id)
                    self.tables[table_var_name] = conditional
                    return z3p.Const(table_var_name, hole.range())
            if expression.num_args() == 2:
                expression1 = self.substitute_hole_in_expression(expression.arg(0), pairs)
                expression2 = self.substitute_hole_in_expression(expression.arg(1), pairs)
                operator = expression.decl()
                return operator(expression1, expression2)
        return expression

    def transition_by_name(self, name):
        return next(t for t in self.edges(data=True) if t[2]['name'] == name)

    def transition_condition(self,
                             transition_name,
                             state):
        """ The values of the variables in the state can be symbolic.
        """
        guard = self.transition_by_name(transition_name)[2]['guard']
        return util.evaluate_expression(guard, {}, state)

    def update_channel_edges(self):
        self.channel_edges = {}
        for edge in self.edges(data=True):
            data = edge[2]
            if data['kind'] == 'input' or data['kind'] == 'output':
                channel = data['channel']
                if channel in self.channel_edges:
                    self.channel_edges[channel].append(edge)
                else:
                    self.channel_edges[channel] = [edge]

    def update_transition_variables(self):
        self.transition_variables = {}
        for _, _, data in self.edges(data=True):
            name = data['name']
            vs = {}
            self.transition_variables[name] = vs
            guard = data['guard']
            if guard is not None:
                vs['guard'] = self.variables_ufunctions_in_expression(data['guard'])
            channel_expressions = data['channel_expression']
            for channel_expression in channel_expressions:
                vs['channel_expression'] = [self.variables_ufunctions_in_expression(ce)
                                            for ce in channel_expressions]
            updates = data['update']
            vs['rhs_update'] = [self.variables_ufunctions_in_expression(rhs) for _, rhs in updates]

    def variables_ufunctions_in_expression(self, expression):
        variables_in_expression = z3p.variables_in_expression(expression)
        variables, ufunctions = [], []
        for v in variables_in_expression:
            if any(v == v2 for v2 in self.variables) or any(v == c for c in self.input_channels):
                variables.append(v)
            else:
                ufunctions.append(v)
        return (variables, ufunctions)


if __name__ == '__main__':
    """Pass the module path and the function to call in the module
    and as a result I will draw the automaton.
    """
    import argparse
    import os
    import sys

    parser = argparse.ArgumentParser(description='Draw automaton.')
    parser.add_argument('module_path', help='python module file that contains automaton function')
    parser.add_argument('automaton_function', help='function in module that creates automaton')
    parser.add_argument('--transition_labels', '-t', action='store_true', help='add transition labels')
    args = parser.parse_args()
    module = util.import_module(args.module_path)
    if module is None:
        sys.stderr.write('File {} does not exist.\n'.format(args.module_path))
        parser.print_help()
        sys.exit(1)
    try:
        function = getattr(module, args.automaton_function)
    except AttributeError:
        sys.stderr.write('Function {} was not found in {}\n'.format(args.automaton_function, args.module_path))
        parser.print_help()
        sys.exit(1)
    automaton = function()
    automaton.draw(transition_labels=args.transition_labels)
    print 'Automaton {} written in tmp/temp.png'.format(automaton)
    os.system('open {}'.format(util.absolute_filename_in_tmp('temp.png')))
