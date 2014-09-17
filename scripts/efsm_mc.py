import os.path

import model
import util
import z3p


class Model(model.Model):
    def array_channel_size(self, channel):
        assert channel.sort_kind() == z3p.Z3_ARRAY_SORT
        a = next(a for a in self.automata if channel in a.output_channels)
        size = None
        for _, _, data in a.edges(data=True):
            if data['kind'] == 'output' and channel == data['channel'] and len(data['channel_expression']) > 0:
                size = len(data['channel_expression'])
        return size

    def automata_definitions(self):
        return '\n'.join(self.automaton_definitions(a) for a in self.automata)

    def automaton_definitions(self, a):
        ret_val = []
        a_name = to_camel_case(a.name)
        ret_val.append('auto {automaton} = TheLTS->MakeGenEFSM("{automaton}", {{}}, TheLTS->MakeTrue(), LTSFairnessType::None)'.format(automaton=a_name))
        for location in a.nodes():
            ret_val.append('{}->AddState("{}")'.format(a_name, to_camel_case(location)))
        ret_val.append('{}->FreezeStates()'.format(a_name))
        for variable in a.variables:
            variable_name = to_camel_case(variable.decl().name())
            variable_type = self.efsm_variable_type(variable)
            ret_val.append('{}->AddVariable("{}", {})'.format(a_name, variable_name, variable_type))
            ret_val.append('auto {}Exp = TheLTS->MakeVar("{}", {})'.format(variable_name, variable_name, variable_type))
        ret_val.append('{}->FreezeVars()'.format(a_name))
        for c in a.input_channels:
            s = '{}->AddInputMsg({}Port)'.format(a_name, to_camel_case(c.decl().name()))
            ret_val.append(s)
        for c in a.output_channels:
            s = '{}->AddOutputMsg({}Port)'.format(a_name, to_camel_case(c.decl().name()))
            ret_val.append(s)

        for start, end, data in a.edges(data=True):
            for line in self.translate_edge((start, end, data)):
                ret_val.append(line)
        ret_val = ['    ' + line + ';' for line in ret_val]
        return '\n'.join(ret_val)

    def automaton_with_variable(self, variable):
        return next((a for a in self.automata if variable in a.variables), None)

    def automaton_state_location(self, a):
        return 'TheLTS->MakeOp(LTSOps::OpField, {}, TheLTS->MakeVar("state", TheLTS->MakeFieldAccessType()))'.format(self.automaton_statevar(a), a)

    def automaton_state_variable(self, a, v):
        return 'TheLTS->MakeOp(LTSOps::OpField, {}, TheLTS->MakeVar("{}", TheLTS->MakeFieldAccessType()))'.format(self.automaton_statevar(a), to_camel_case(v.decl().name()))

    def automaton_statevar(self, a):
        return 'TheLTS->MakeVar("{a}", TheLTS->GetEFSMType("{a}"))'.format(a=to_camel_case(a.name))

    def channel_definition(self, c):
        ret_val = []
        c_name = to_camel_case(c.decl().name())
        ret_val.append('vector<pair<string, ExprTypeRef>> {}Fields'.format(c_name))
        if c.sort_kind() == z3p.Z3_ARRAY_SORT:
            for i in range(self.array_channel_size(c)):
                low, high = self.channel_range(c[i])
                ret_val.append('{}Fields.push_back(make_pair("Field{}", TheLTS->MakeRangeType({}, {})))'.format(c_name, i, low, high))
        else:
            low, high = self.channel_range(c)
            ret_val.append('{}Fields.push_back(make_pair("Field{}", TheLTS->MakeRangeType({}, {})))'.format(c_name, 0, low, high))
        ret_val.append('auto {c}Port = TheLTS->MakeMsgType("{c}Port", {c}Fields, false)'.format(c=c_name))
        ret_val.append('auto {c}PortExp = TheLTS->MakeVar("{c}Port", {c}Port)'.format(c=c_name))
        ret_val = ['    ' + line + ';' for line in ret_val]
        return '\n'.join(ret_val)

    def channel_definitions(self):
        return '\n'.join([self.channel_definition(c) for c in self.channels])

    def channel_range(self, channel):
        if isinstance(channel, z3p.ArrayRef):
            channel_ranges = []
            for i in range(self.array_channel_size(channel)):
                variables = self.reading_variables_for_channel_field(channel[i])
                ranges = set([self.automaton_with_variable(v).variable_ranges[v] for v in variables])
                assert len(ranges) == 1
                channel_ranges.append(ranges)
            return channel_ranges
            # return "{}Type".format(to_camel_case(channel.decl().name()))
        elif channel.sort() == util.UNIT_SORT:
            return (0, 0)
        elif channel.sort() == z3p.BoolSort():
            return (0, 1)
        elif channel.sort() == z3p.IntSort():
            variables = self.reading_variables_for_channel_or_channel_field(channel)
            ranges = set([self.automaton_with_variable(v).variable_ranges[v] for v in variables])
            assert len(ranges) == 1
            return ranges.pop()
        else:
            raise NotImplementedError("Channel type cannot be defined for channel {} with sort {}.".format(channel, channel.sort()))

    def efsm_channel_type(self, channel):
        if isinstance(channel, z3p.ArrayRef):
            return "{}Type".format(to_camel_case(channel.decl().name()))
        if channel.sort() == util.UNIT_SORT:
            return "RangeType"
        elif channel.sort() == z3p.BoolSort():
            return "Range01Type"
        elif channel.sort() == z3p.IntSort():
            variables = self.reading_variables_for_channel(channel)
            for v in variables:
                a = next(a for a in self.automata if v in a.variables)
                low, high = a.variable_ranges[v]
                return "Range{}{}Type".format(low, high)
        else:
            raise NotImplementedError("Channel type cannot be defined for channel {} with sort {}.".format(channel, channel.sort()))

    def efsm_variable_type(self, variable):
        a = next(a for a in self.automata if variable in a.variables)
        if variable.sort() == z3p.BoolSort():
            return 'TheLTS->MakeRangeType(0, 1)'
        elif variable.sort() == z3p.IntSort():
            low, high = a.variable_ranges[variable]
            return "TheLTS->MakeRangeType({}, {})".format(low, high)
        else:
            raise NotImplementedError("Channel type cannot be defined for channel {} with sort {}.".format(variable, variable.sort()))

    def __init__(self, automata):
        super(Model, self).__init__(automata)
        self.automata_variables = [v for a in self.automata for v in a.variables]

    def initial_automaton_states(self, a):
        return ''

    def initial_automaton_updates(self, a):
        ret_val = []
        for variable in a.variables:
            ret_val.append((self.automaton_state_variable(a, variable), self.translate_expression(a.initials[variable])))
        ret_val.append((self.automaton_state_location(a), 'TheLTS->MakeVal("{}", {}->GetType())'.format(to_camel_case(a.initial_location), self.automaton_state_location(a))))
        ret_val = ['new LTSAssignSimple({}, {})'.format(lhs, rhs)
                   for lhs, rhs in ret_val]
        return ret_val

    def initial_states(self):
        return '\n'.join(self.initial_automaton_states(a) for a in self.automata)

    def initial_updates(self):
        ret_val = []
        for a in self.automata:
            for update in self.initial_automaton_updates(a):
                ret_val.append(update)
        ret_val = ['InitUpdates.push_back({})'.format(arg) for arg in ret_val]
        return '\n'.join('    {};'.format(arg) for arg in ret_val)

    def invariants(self):
        return ''

    def reading_variables_for_channel_or_channel_field(self, channel_or_channel_field):
        if channel_or_channel_field.decl().kind() == z3p.Z3_OP_SELECT:
            return self.reading_variables_for_channel_field(channel_or_channel_field)
        else:
            return self.reading_variables_for_channel(channel_or_channel_field)

    def reading_variables_for_channel(self, channel):
        assert channel.sort_kind() != z3p.Z3_ARRAY_SORT
        variables = set([])
        for a in self.readers[channel]:
            for _, _, data in a.edges(data=True):
                if data['kind'] == 'input' and channel == data['channel'] and len(data['update']) > 0:
                    lhs, _ = data['update'][0]
                    variables.add(lhs)
        return list(variables)

    def reading_variables_for_channel_field(self, channel_field):
        assert channel_field.decl().kind() == z3p.Z3_OP_SELECT
        field_num = channel_field.arg(1).as_long()
        variables = set([])
        channel = channel_field.arg(0)
        for a in self.readers[channel]:
            for _, _, data in a.edges(data=True):
                if data['kind'] == 'input' and channel == data['channel'] and len(data['update']) > 0:
                    lhs, _ = data['update'][field_num]
                    variables.add(lhs)
        return list(variables)

    def __str__(self):
        folder = os.path.dirname(os.path.realpath(__file__))
        template_path = os.path.join(folder, 'efsm_mc_template.txt')
        f = open(template_path, 'r')
        s = f.read()
        ret_val = ''
        return s.format(channel_definitions=self.channel_definitions(),
                        automata_definitions=self.automata_definitions(),
                        invariants=self.invariants(),
                        initial_updates=self.initial_updates())

    def to_file(self, filename):
        with open(filename, 'w') as f:
            f.write(self.__str__())

    def translate_binary_expression(self, e):
        e1 = self.translate_expression(e.arg(0))
        e2 = self.translate_expression(e.arg(1))
        ops_map = {z3p.Z3_OP_LT: "LTSOps::OpLT",
                   z3p.Z3_OP_GT: "LTSOps::OpGT",
                   z3p.Z3_OP_GE: "LTSOps::OpGE",
                   z3p.Z3_OP_EQ: "LTSOps::OpEQ",
                   z3p.Z3_OP_ADD: "LTSOps::OpADD",
                   z3p.Z3_OP_MOD: "LTSOps::OpMOD"}
        if e.decl().kind() in ops_map:
            return 'TheLTS->MakeOp({}, {}, {})'.format(ops_map[e.decl().kind()], e1, e2)
        elif e.decl().kind() == z3p.Z3_OP_DISTINCT:
            eq_e = 'TheLTS->MakeOp(LTSOps::OpEQ, {}, {})'.format(e1, e2)
            return 'TheLTS->MakeOp(LTSOps::OpNOT, {})'.format(eq_e)

    def translate_channel_reference(self, channel_reference):
        ret_val = 'TheLTS->MakeOp(LTSOps::OpField, {}PortExp, TheLTS->MakeVar("Field{}", TheLTS->MakeFieldAccessType()))'
        if channel_reference.decl().kind() != z3p.Z3_OP_SELECT:
            channel = channel_reference
            index = 0
        else:
            channel = channel_reference.arg(0)
            index = channel_reference.arg(1).as_long()
        channel_name = to_camel_case(channel.decl().name())
        return ret_val.format(channel_name, index)

    def translate_edge(self, edge):
        start, end, data = edge
        channel = data['channel']
        new_data = {}
        a_name = to_camel_case(data['automaton'].name)
        new_data['automaton'] = a_name
        new_data['start'] = to_camel_case(start)
        new_data['end'] = to_camel_case(end)
        new_data['guard'] = self.translate_expression(data['guard'])
        updates = data['update']
        if data['kind'] == 'output':
            channel_expressions = data['channel_expression']
            if isinstance(channel, z3p.ArrayRef):
                for index, expression in enumerate(channel_expressions):
                    updates.append((channel[index], expression))
            else:
                updates.append((channel, channel_expressions[0]))
        new_data['updates'] = self.translate_updates(updates)
        if data['kind'] in ['input', 'output']:
            new_data['channel'] = to_camel_case(channel.decl().name()) + 'Port'
        if data['kind'] == 'input':
            template = '{automaton}->AddInputTransition("{start}", "{end}", {guard}, {updates}, "{channel}", {channel}, {{}})'
        elif data['kind'] == 'output':
            template = '{automaton}->AddOutputTransition("{start}", "{end}", {guard}, {updates}, "{channel}", {channel}, {{}})'
        elif data['kind'] == 'internal':
            template = '{automaton}->AddInternalTransition("{start}", "{end}", {guard}, {updates})'
        return ['// {}'.format(data['name']), template.format(**new_data)]

    def translate_expression(self, e):
        if e == z3p.BoolVal(True):
            return 'TheLTS->MakeTrue()'
        elif e == z3p.BoolVal(False):
            return 'TheLTS->MakeFalse()'
        elif z3p.is_int_value(e):
            return 'TheLTS->MakeVal("{i}", TheLTS->MakeRangeType({i}, {i}))'.format(i=e)
        elif z3p.is_const(e) and e in self.channels:
            return self.translate_channel_reference(e)
        elif z3p.is_const(e):
            return '{}Exp'.format(to_camel_case(e.decl().name()))
        elif e.num_args() == 1:
            return self.translate_unary_expression(e)
        elif e.num_args() == 2 and e.decl().kind() == z3p.Z3_OP_SELECT:
            return self.translate_channel_reference(e)
        elif e.num_args() == 2:
            return self.translate_binary_expression(e)
        elif e.num_args() == 3:
            return self.translate_ternary_expreession(e)

    def translate_ternary_expreession(self, e):
        assert e.decl().kind() == z3p.Z3_OP_ITE
        e0 = self.translate_expression(e.arg(0))
        e1 = self.translate_expression(e.arg(1))
        e2 = self.translate_expression(e.arg(2))
        return 'TheLTS->MakeOp(LTSOps::OpITE, {}, {}, {})'.format(e0, e1, e2)

    def translate_unary_expression(self, e):
        e0 = self.translate_expression(e.arg(0))
        if e.decl().kind() == z3p.Z3_OP_NOT:
            return 'TheLTS->MakeOp(LTSOps::OpNOT, {})'.format(e0)

    def translate_update(self, update):
        lhs, rhs = update
        e_lhs = self.translate_expression(lhs)
        e_rhs = self.translate_expression(rhs)
        return 'new LTSAssignSimple({}, {})'.format(e_lhs, e_rhs)

    def translate_updates(self, updates):
        return '{{ {} }}'.format(', '.join(self.translate_update(update) for update in updates))

    def type_definition(self, efsm_type_name):
        pass


def to_camel_case(name):
    name = name.title()
    name = ''.join(x for x in name if not x == "_")
    return name


import main

if __name__ == '__main__':
    main.efsm_mc_main()
