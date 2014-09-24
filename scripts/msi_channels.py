import automaton
import util
import z3p

TRUE = z3p.BoolVal(True)

def INT(x):
    return z3p.IntVal(x)

ZERO = z3p.IntVal(0)

ONE = z3p.IntVal(1)

NUM_ADDRESSES = 1
NUM_CACHES = 2
NUM_DIRECTORIES = 1
NUM_VALUES = 1


LDMsg = {}
STMsg = {}
EVMsg = {}
FwdGetXMsgP = {}
FwdGetSMsgP = {}
InvAckMsg = {}
InvAckMsgP = {}
DataMsgD2CP = {}
WBAckMsgP = {}
DataMsgC2C = {}
DataMsgC2CP = {}
LDAckMsg = {}
STAckMsg = {}
EVAckMsg = {}
UnblockSMsg = {}
UnblockEMsg = {}
GetXMsg = {}
GetSMsg = {}
WBMsg = {}

GetXMsgP = {}
GetSMsgP = {}
WBMsgP = {}
UnblockSMsgP = {}
UnblockEMsgP = {}

FwdGetXMsg = {}
FwdGetSMsg = {}
DataMsgD2C = {}
WBAckMsg = {}

for c in range(NUM_CACHES):
    LDMsg[c] = {}
    STMsg[c] = {}
    EVMsg[c] = {}
    FwdGetXMsgP[c] = {}
    FwdGetSMsgP[c] = {}
    InvAckMsgP[c] = {}
    DataMsgD2CP[c] = {}
    WBAckMsgP[c] = {}
    LDAckMsg[c] = {}
    STAckMsg[c] = {}
    EVAckMsg[c] = {}
    UnblockSMsg[c] = {}
    UnblockEMsg[c] = {}
    GetXMsg[c] = {}
    GetSMsg[c] = {}
    WBMsg[c] = {}

    GetXMsgP[c] = {}
    GetSMsgP[c] = {}
    WBMsgP[c] = {}
    UnblockSMsgP[c] = {}
    UnblockEMsgP[c] = {}

    FwdGetXMsg[c] = {}
    FwdGetSMsg[c] = {}
    DataMsgD2C[c] = {}
    WBAckMsg[c] = {}

    for d in range(NUM_DIRECTORIES):
        LDMsg[c][d] = {}
        STMsg[c][d] = {}
        EVMsg[c][d] = {}
        FwdGetXMsgP[c][d] = {}
        FwdGetSMsgP[c][d] = {}
        InvAckMsgP[c][d] = {}
        DataMsgD2CP[c][d] = {}
        WBAckMsgP[c][d] = {}
        LDAckMsg[c][d] = {}
        STAckMsg[c][d] = {}
        EVAckMsg[c][d] = {}
        UnblockSMsg[c][d] = {}
        UnblockEMsg[c][d] = {}
        GetXMsg[c][d] = {}
        GetSMsg[c][d] = {}
        WBMsg[c][d] = {}

        GetXMsgP[c][d] = {}
        GetSMsgP[c][d] = {}
        WBMsgP[c][d] = {}
        UnblockSMsgP[c][d] = {}
        UnblockEMsgP[c][d] = {}

        FwdGetXMsg[c][d] = {}
        FwdGetSMsg[c][d] = {}
        DataMsgD2C[c][d] = {}
        WBAckMsg[c][d] = {}


        for a in range(NUM_ADDRESSES):
            suffix = '_{}_{}_{}'.format(c, d, a)
            LDMsg[c][d][a] = util.new_variable('LDMsg' + suffix, 'unit')
            STMsg[c][d][a] = z3p.Int('STMsg' + suffix)
            EVMsg[c][d][a] = util.new_variable('EVMsg' + suffix, 'unit')
            FwdGetXMsgP[c][d][a] = z3p.Int('FwdGetXMsgP' + suffix)
            FwdGetSMsgP[c][d][a] = z3p.Int('FwdGetSMsgP' + suffix)
            DataMsgD2CP[c][d][a] = z3p.Array('DataMsgD2CP' + suffix, z3p.IntSort(), z3p.IntSort())
            WBAckMsgP[c][d][a] = util.new_variable('WBAckMsgP' + suffix, 'unit')
            LDAckMsg[c][d][a] = z3p.Int('LDAckMsg' + suffix)
            STAckMsg[c][d][a] = z3p.Int('STAckMsg' + suffix)
            EVAckMsg[c][d][a] = util.new_variable('EVAckMsg' + suffix, 'unit')
            UnblockSMsg[c][d][a] = util.new_variable('UnblockSMsg' + suffix, 'unit')
            UnblockEMsg[c][d][a] = util.new_variable('UnblockEMsg' + suffix, 'unit')
            GetXMsg[c][d][a] = util.new_variable('GetXMsg' + suffix, 'unit')
            GetSMsg[c][d][a] = util.new_variable('GetSMsg' + suffix, 'unit')
            WBMsg[c][d][a] = z3p.Int('WBMsg' + suffix)

            GetXMsgP[c][d][a] = util.new_variable('GetXMsgP' + suffix, 'unit')
            GetSMsgP[c][d][a] = util.new_variable('GetSMsgP' + suffix, 'unit')
            WBMsgP[c][d][a] = z3p.Int('WBMsgP' + suffix)
            UnblockSMsgP[c][d][a] = util.new_variable('UnblockSMsgP' + suffix, 'unit')
            UnblockEMsgP[c][d][a] = util.new_variable('UnblockEMsgP' + suffix, 'unit')

            FwdGetXMsg[c][d][a] = z3p.Int('FwdGetXMsg' + suffix)
            FwdGetSMsg[c][d][a] = z3p.Int('FwdGetSMsg' + suffix)
            DataMsgD2C[c][d][a] = z3p.Array('DataMsgD2C' + suffix, z3p.IntSort(), z3p.IntSort())
            WBAckMsg[c][d][a] = util.new_variable('WBAckMsg' + suffix, 'unit')


for c in range(NUM_CACHES):
    DataMsgC2CP[c] = {}
    DataMsgC2C[c] = {}
    DataMsgC2CP[c] = {}
    InvAckMsg[c] = {}
    InvAckMsgP[c] = {}
    for c2 in range(NUM_CACHES):
        if c2 != c:
            DataMsgC2C[c][c2] = {}
            DataMsgC2CP[c][c2] = {}
            InvAckMsg[c][c2] = {}
            InvAckMsgP[c][c2] = {}
            for d in range(NUM_DIRECTORIES):
                DataMsgC2C[c][c2][d] = {}
                DataMsgC2CP[c][c2][d] = {}
                InvAckMsg[c][c2][d] = {}
                InvAckMsgP[c][c2][d] = {}
                for a in range(NUM_ADDRESSES):
                    suffix = '_{}_{}_{}_{}'.format(c, c2, d, a)
                    DataMsgC2C[c][c2][d][a] = z3p.Int('DataMsgC2C' + suffix)
                    InvAckMsg[c][c2][d][a] = util.new_variable('InvAckMsg' + suffix, 'unit')
                    DataMsgC2CP[c][c2][d][a] = z3p.Int('DataMsgC2CP' + suffix)
                    InvAckMsgP[c][c2][d][a] = util.new_variable('InvAckMsgP' + suffix, 'unit')


def lossless_non_duplicating_blocking_unordered_channel(name, inputs, outputs, data_ranges, port_number_of_fields, capacity):
    variables = []
    initial_values = []
    variable_ranges = {}
    counters = []
    print inputs
    print outputs
    for input in inputs:
        input_name = input.decl().name()
        input_counter = z3p.Int(input_name + '_counter')
        variables.append(input_counter)
        initial_values.append(ZERO)
        variable_ranges[input_counter] = (0, capacity)
        counters.append(input_counter)
        # if the type of the port is unit then all we need is the counter
        if not isinstance(input, z3p.ArrayRef):
            if input in data_ranges:
                for i in range(capacity):
                    variable = z3p.Int('{}_cell{}'.format(input.decl().name(), i))
                    variables.append(variable)
                    initial_values.append(ZERO)
                    variable_ranges[variable] = data_ranges[input]
        else:
            for i in range(capacity):
                for j in range(port_number_of_fields[input]):
                    variable = z3p.Int('{}_field_{}_cell{}'.format(input.decl().name(), j, i))
                    variables.append(variable)
                    initial_values.append(ZERO)
                    variable_ranges[variable] = data_ranges[input[j]]

    locations = ['initial']
    transitions = []

    for input, output in zip(inputs, outputs):
        input_name = input.decl().name()
        output_name = output.decl().name()
        input_counter = z3p.Int(input_name + '_counter')
        for i in range(capacity):
            updates = []
            if not isinstance(input, z3p.ArrayRef):
                if input in data_ranges:
                    variable = z3p.Int('{}_cell{}'.format(input.decl().name(), i))
                    updates.append((variable, input))
            else:
                for j in range(port_number_of_fields[input]):
                    variable = z3p.Int('{}_field_{}_cell{}'.format(input.decl().name(), j, i))
                    updates.append((variable, input[j]))
            last = counters[0]
            for counter in counters[1:]:
                last = z3p.Sum(last, counter)
            counters_sum = last
            transitions.append(('t_{}'.format(input_name),
                                'initial',
                                input,
                                z3p.And(counters_sum < z3p.IntVal(capacity), z3p.Eq(input_counter, i)),
                                [(input_counter, input_counter + 1)] + updates,
                                'initial'))
            if i > 0:
                updates = []
                channel_outputs = []
                if not isinstance(input, z3p.ArrayRef):
                    if input in data_ranges:
                        variable = z3p.Int('{}_cell{}'.format(input.decl().name(), i))
                        channel_outputs.append(variable)
                        updates.append((variable, ZERO))
                    else:
                        channel_outputs.append(ZERO)
                else:
                    for j in range(port_number_of_fields[input]):
                        variable = z3p.Int('{}_field_{}_cell{}'.format(input.decl().name(), j, i))
                        channel_outputs.append(variable)
                        updates.append((variable, ZERO))

                transitions.append(('t_{}'.format(output_name),
                                    'initial',
                                    output,
                                    channel_outputs,
                                    z3p.Eq(input_counter, z3p.IntVal(i)),
                                    [(input_counter, input_counter - 1)] + updates,
                                    'initial'))
    return automaton.SymbolicAutomaton('{}_channel'.format(name),
                                       locations,
                                       'initial',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=inputs,
                                       output_channels=outputs,
                                       variable_ranges=variable_ranges)


def lossless_non_duplicating_blocking_capacity_one_channel(name, inputs, outputs, data_ranges, port_number_of_fields):
    locations = ['empty'] + ['full_{}'.format(i) for i in inputs]
    variables = []
    initial_values = []
    variable_ranges = {}
    for input_channel in inputs:
        input_channel_name = input_channel.decl().name()
        if not isinstance(input_channel, z3p.ArrayRef):
            if input_channel in data_ranges:
                variable = z3p.Int('{}_cell'.format(input_channel_name))
                variables.append(variable)
                initial_values.append(ZERO)
                variable_ranges[variable] = data_ranges[input_channel]
        else:
            assert input_channel in port_number_of_fields
            for field in range(port_number_of_fields[input_channel]):
                variable = z3p.Int('{}_cell_{}'.format(input_channel_name, field))
                variables.append(variable)
                initial_values.append(ZERO)
                variable_ranges[variable] = data_ranges[input_channel[field]]
    transitions = []
    for i, (input_channel, output_channel) in enumerate(zip(inputs, outputs)):
        input_channel_name = input_channel.decl().name()
        input_transition_name = 't_input_{}'.format(input_channel_name)
        output_transition_name = 't_output_{}'.format(input_channel_name)
        full_state = 'full_{}'.format(input_channel_name)
        if isinstance(input_channel, z3p.ArrayRef):
            print "array channel is {}".format(input_channel)
            tmp_variables = [z3p.Int('{}_cell_{}'.format(input_channel_name, field)) for field in range(port_number_of_fields[input_channel])]
            updates = zip(tmp_variables, [input_channel[i] for i in range(port_number_of_fields[input_channel])])
            transitions.append((input_transition_name, 'empty', input_channel, TRUE, updates, full_state))
            updates = zip(tmp_variables, port_number_of_fields[input_channel] * [ZERO])
            transitions.append((output_transition_name, full_state, output_channel, tmp_variables, TRUE, updates, 'empty'))
        elif input_channel in data_ranges:
            variable = z3p.Int('{}_cell'.format(input_channel_name))
            transitions.append((input_transition_name, 'empty', input_channel, TRUE, [(variable, input_channel)], full_state))
            transitions.append((output_transition_name, full_state, output_channel, [variable], TRUE, [(variable, ZERO)], 'empty'))
        else:
            transitions.append((input_transition_name, 'empty', input_channel, TRUE, [], full_state))
            transitions.append((output_transition_name, full_state, output_channel, [ZERO], TRUE, [], 'empty'))

    return automaton.SymbolicAutomaton('{}_channel'.format(name),
                                       locations,
                                       'empty',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=inputs,
                                       output_channels=outputs,
                                       variable_ranges=variable_ranges)
