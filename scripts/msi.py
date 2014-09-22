import automaton
import util
import z3p



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
        initial_values.append(z3p.IntVal(0))
        variable_ranges[input_counter] = (0, capacity)
        counters.append(input_counter)
        # if the type of the port is unit then all we need is the counter
        if not isinstance(input, z3p.ArrayRef):
            if input in data_ranges:
                for i in range(capacity):
                    variable = z3p.Int('{}_cell{}'.format(input.decl().name(), i))
                    variables.append(variable)
                    initial_values.append(z3p.IntVal(0))
                    variable_ranges[variable] = data_ranges[input]
        else:
            for i in range(capacity):
                for j in range(port_number_of_fields[input]):
                    variable = z3p.Int('{}_field_{}_cell{}'.format(input.decl().name(), j, i))
                    variables.append(variable)
                    initial_values.append(z3p.IntVal(0))
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
                    print "pritnint"
                    print updates
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
                        updates.append((variable, z3p.IntVal(0)))
                    else:
                        channel_outputs.append(z3p.IntVal(0))
                else:
                    for j in range(port_number_of_fields[input]):
                        variable = z3p.Int('{}_field_{}_cell{}'.format(input.decl().name(), j, i))
                        channel_outputs.append(variable)
                        updates.append((variable, z3p.IntVal(0)))

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
    for input in inputs:
        input_name = input.decl().name()
        if not isinstance(input, z3p.ArrayRef):
            if input in data_ranges:
                variable = z3p.Int('{}_cell'.format(input_name))
                variables.append(variable)
                initial_values.append(z3p.IntVal(0))
                variable_ranges[variable] = data_ranges[input]
        else:
            assert input in port_number_of_fields
            for field in range(port_number_of_fields[input]):
                variable = z3p.Int('{}_cell_{}'.format(input_name, field))
                variables.append(variable)
                initial_values.append(z3p.IntVal(0))
                variable_ranges[variable] = data_ranges[input[field]]
    print variables
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
            transitions.append((input_transition_name, 'empty', input_channel, z3p.BoolVal(True), updates, full_state))
            updates = zip(tmp_variables, port_number_of_fields[input_channel] * [z3p.IntVal(0)])
            transitions.append((output_transition_name, full_state, output_channel, tmp_variables, z3p.BoolVal(True), updates, 'empty'))
        elif input_channel in data_ranges:
            variable = z3p.Int('{}_cell'.format(input_channel_name))
            transitions.append((input_transition_name, 'empty', input_channel, z3p.BoolVal(True), [(variable, input_channel)], full_state))
            transitions.append((output_transition_name, full_state, output_channel, [variable], z3p.BoolVal(True), [(variable, z3p.IntVal(0))], 'empty'))
        else:
            transitions.append((input_transition_name, 'empty', input_channel, z3p.BoolVal(True), [], full_state))
            transitions.append((output_transition_name, full_state, output_channel, [z3p.IntVal(0)], z3p.BoolVal(True), [], 'empty'))

    for t in transitions:
        print t
    return automaton.SymbolicAutomaton('{}_channel'.format(name),
                                       locations,
                                       'empty',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=inputs,
                                       output_channels=outputs,
                                       variable_ranges=variable_ranges)


def cache(cache_id, num_caches=2, num_values=2, num_addresses=2, num_directories=1):
    transitions = []
    variables = []
    variable_ranges = {}
    DataBlk = z3p.Int('DataBlk_{}'.format(cache_id))
    AckCounter = z3p.Int('AckCounter_{}'.format(cache_id))
    PendingWrite = z3p.Int('PendingWrite_{}'.format(cache_id))
    FwdToCache = z3p.Int('FwdToCache_{}'.format(cache_id))
    NumAcksTemp = z3p.Int('NumAcksTemp_{}'.format(cache_id))
    variable_ranges = {DataBlk: (0, num_values - 1),
                       AckCounter: (-num_caches, num_caches),
                       PendingWrite: (0, num_values - 1),
                       FwdToCache: (0, num_caches - 1),
                       NumAcksTemp: (0, num_caches)}
    variables = [DataBlk, AckCounter, PendingWrite, FwdToCache, NumAcksTemp]
    initial_values = [z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0)]
    # ??? initial_values

    input_channels = []
    # input messages
    LDMsg = {}
    STMsg = {}
    EVMsg = {}
    FwdGetXMsgP = {}
    FwdGetSMsgP = {}
    InvAckMsg = {}
    InvAckMsgP = {}
    DataMsgD2CP = {}
    WBAckMsgP = {}
    DataMsgC2CP = {}
    for d in range(num_directories):
        LDMsg[d] = {}
        STMsg[d] = {}
        EVMsg[d] = {}
        FwdGetXMsgP[d] = {}
        FwdGetSMsgP[d] = {}
        InvAckMsgP[d] = {}
        DataMsgD2CP[d] = {}
        DataMsgC2CP[d] = {}
        WBAckMsgP[d] = {}
        for a in range(num_addresses):
            suffix = '_{}_{}_{}'.format(cache_id, d, a)
            LDMsg[d][a] = util.new_variable('LDMsg' + suffix, 'unit')
            input_channels.append(LDMsg[d][a])
            STMsg[d][a] = z3p.Int('STMsg' + suffix)
            input_channels.append(STMsg[d][a])
            EVMsg[d][a] = util.new_variable('EVMsg' + suffix, 'unit')
            input_channels.append(EVMsg[d][a])
            FwdGetXMsgP[d][a] = z3p.Int('FwdGetXMsgP' + suffix)
            input_channels.append(FwdGetXMsgP[d][a])
            FwdGetSMsgP[d][a] = z3p.Int('FwdGetSMsgP' + suffix)
            input_channels.append(FwdGetSMsgP[d][a])
            DataMsgD2CP[d][a] = z3p.Array('DataMsgD2CP' + suffix, z3p.IntSort(), z3p.IntSort())
            input_channels.append(DataMsgD2CP[d][a])
            WBAckMsgP[d][a] = z3p.Int('WBAckMsgP' + suffix)
            input_channels.append(WBAckMsgP[d][a])
            InvAckMsgP[d][a] = {}
            DataMsgC2CP[d][a] = {}
            for other_c in range(num_caches):
                if other_c != cache_id:
                    suffix2 = '_{}{}'.format(other_c, suffix)
                    InvAckMsgP[d][a][other_c] = util.new_variable('InvAckMsgP' + suffix2, 'unit')
                    input_channels.append(InvAckMsgP[d][a][other_c])
                    DataMsgC2CP[d][a][other_c] = z3p.Int('DataMsgC2CP' + suffix2)
                    input_channels.append(DataMsgC2CP[d][a][other_c])

    output_channels = []
    # output messages
    LDAckMsg = {}
    STAckMsg = {}
    EVAckMsg = {}
    UnblockSMsg = {}
    UnblockEMsg = {}
    InvAckMsg = {}
    GetXMsg = {}
    GetSMsg = {}
    WBMsg = {}
    DataMsgC2C = {}
    for d in range(num_directories):
        LDAckMsg[d] = {}
        STAckMsg[d] = {}
        EVAckMsg[d] = {}
        UnblockSMsg[d] = {}
        UnblockEMsg[d] = {}
        InvAckMsg[d] = {}
        GetXMsg[d] = {}
        GetSMsg[d] = {}
        WBMsg[d] = {}
        DataMsgC2C[d] = {}
        for a in range(num_addresses):
            suffix = '_{}_{}_{}'.format(cache_id, d, a)
            LDAckMsg[d][a] = util.new_variable('LDAckMsg' + suffix, 'unit')
            output_channels.append(LDAckMsg[d][a])
            STAckMsg[d][a] = z3p.Int('STAckMsg' + suffix)
            output_channels.append(STAckMsg[d][a])
            EVAckMsg[d][a] = util.new_variable('EVAckMsg' + suffix, 'unit')
            output_channels.append(EVAckMsg[d][a])
            UnblockSMsg[d][a] = util.new_variable('UnblockSMsg' + suffix, 'unit')
            output_channels.append(UnblockSMsg[d][a])
            UnblockEMsg[d][a] = util.new_variable('UnblockEMsg' + suffix, 'unit')
            output_channels.append(UnblockEMsg[d][a])

            GetXMsg[d][a] = util.new_variable('GetXMsg' + suffix, 'unit')
            output_channels.append(GetXMsg[d][a])
            GetSMsg[d][a] = util.new_variable('GetSMsg' + suffix, 'unit')
            output_channels.append(GetSMsg[d][a])
            WBMsg[d][a] = z3p.Int('WBMsg' + suffix)
            output_channels.append(WBMsg[d][a])

            InvAckMsg[d][a] = {}
            DataMsgC2C[d][a] = {}
            for other_c in range(num_caches):
                if other_c != cache_id:
                    suffix2 = '_{}_{}_{}_{}'.format(other_c, cache_id, d, a)
                    InvAckMsg[d][a][other_c] = util.new_variable('InvAckMsg' + suffix2, 'unit')
                    output_channels.append(InvAckMsg[d][a][other_c])
                    DataMsgC2C[d][a][other_c] = z3p.Int('DataMsgC2C' + suffix2)
                    output_channels.append(DataMsgC2C[d][a][other_c])

    c = cache_id

    for d in range(num_directories):
        for a in range(num_addresses):
            # // Transitions from I for Ext events
            # C_I on LDMsg[c][d][a] {} -> send GetSMsg[c][d][a] {} -> C_IS;
            # C_I on STMsg[c][d][a] (v) { PendingWrite := v; } ->
            # send GetXMsg[c][d][a] {} -> C_IM;
            # C_I on EVMsg[c][d][a] {} -> send EVAckMsg[c][d][a] -> C_I;
            transitions.append((None, 'C_I', LDMsg[d][a], z3p.BoolVal(True), [], 'C_I2'))
            transitions.append((None, 'C_I2', GetSMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_IS'))
            transitions.append((None, 'C_I', STMsg[d][a], z3p.BoolVal(True), [(PendingWrite, STMsg[d][a])], 'C_I3'))
            transitions.append((None, 'C_I3', GetXMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_IM'))
            transitions.append((None, 'C_I', EVMsg[d][a], z3p.BoolVal(True), [], 'C_I4'))
            transitions.append((None, 'C_I4', EVAckMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_I'))

            # // Transitions for I on Fwd events
            # C_I on FwdGetSMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] { FwdToCache := undef; } -> C_I;
            transitions.append((None, 'C_I', FwdGetSMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetSMsgP[d][a])], 'C_I5'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_I5', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.IntVal(0))], 'C_I'))


            # // Transitions from S on Ext Events
            # C_S on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) -> C_S;
            transitions.append((None, 'C_S', LDMsg[d][a], z3p.BoolVal(True), [], 'C_S2'))
            transitions.append((None, 'C_S2', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_S'))

            # C_S on STMsg[c][d][a] (v) { PendingWrite := v; } ->
            # send GetXMsg[c][d][a] {} -> C_SM;
            transitions.append((None, 'C_S', STMsg[d][a], z3p.BoolVal(True), [(PendingWrite, STMsg[d][a])], 'C_S3'))
            transitions.append((None, 'C_S3', GetXMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_SM'))

            # C_S on EVMsg[c][d][a] {} ->
            # send EVAckMsg[c][d][a] { DataBlk := undef; } -> C_I;
            transitions.append((None, 'C_S', EVMsg[d][a], z3p.BoolVal(True), [], 'C_S4'))
            transitions.append((None, 'C_S4', EVAckMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [(DataBlk, z3p.IntVal(0))], 'C_SM'))

            # // Transitions from S on Fwd events
            # C_S on FwdGetXMsg[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] { DataBlk := undef } -> C_I;
            transitions.append((None, 'C_S', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_S5'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_S5', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.IntVal(0))], 'C_I'))

            # // Transitions from M on Ext events
            # C_M on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_M;
            transitions.append((None, 'C_M', LDMsg[d][a], z3p.BoolVal(True), [], 'C_M2'))
            transitions.append((None, 'C_M2', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))
            # C_M on STMsg[c][d][a] (v) { DataBlk := v; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            transitions.append((None, 'C_M', STMsg[d][a], z3p.BoolVal(True), [(DataBlk, STMsg[d][a])], 'C_M3'))
            transitions.append((None, 'C_M3', STAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))

            # C_M on EVMsg[c][d][a] {} -> send WBMsg[c][d][a] (DataBlk) {} -> C_II;
            transitions.append((None, 'C_M', EVMsg[d][a], z3p.BoolVal(True), [], 'C_M4'))
            transitions.append((None, 'C_M4', WBMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_II'))


            # // Transitions from M on Fwd events
            # C_M on FwdGetSMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send DataMsgC2C[c][FwdToCache][d][a] (DataBlk) { FwdToCache := undef; } -> C_S;
            transitions.append((None, 'C_M', FwdGetSMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetSMsgP[d][a])], 'C_M5'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_M5', DataMsgC2C[d][a][other_c], [DataBlk], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.IntVal(0))], 'C_S'))

            # C_M on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send DataMsgC2C[c][FwdToCache][d][a] (DataBlk)
            # { FwdToCache := undef; DataBlk := undef; } -> C_I;
            transitions.append((None, 'C_M', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_M6'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_M6', DataMsgC2C[d][a][other_c], [DataBlk], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.IntVal(0)), (DataBlk, z3p.IntVal(0))], 'C_I'))

            # // Transitions from C_IM on Rsp Events
            # C_IM on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {FwdToCache := undef; } -> C_IM;
            transitions.append((None, 'C_IM', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_IM2'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IM2', InvAckMsg[d][a][other_c], [DataBlk], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [(FwdToCache, z3p.IntVal(0))], 'C_IM'))
            # // Case when another cache gives me data
            # foreach c2 in CacheIDType (!= c2 c) {
            # C_IM on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> C_M;
            # }
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IM', DataMsgC2CP[d][a][other_c], z3p.BoolVal(True), [(DataBlk, DataMsgC2CP[d][a][other_c])], 'C_M'))

            # // Case when data comes from dir ==> Must wait for acks
            # C_IM on DataMsgD2C'[c][d][a] (v, NumAcks)
            # if (!= AckCounter NumAcks)
            # { AckCounter := (- AckCounter NumAcks); DataBlk := v; } -> C_SM;
            # if (= AckCounter NumAcks)
            # { AckCounter := 0; DataBlk := v; } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } -> C_M;
            transitions.append((None, 'C_IM', DataMsgD2CP[d][a], z3p.BoolVal(True), [(DataBlk, DataMsgD2CP[d][a][0]), (NumAcksTemp, DataMsgD2CP[d][a][1])], 'C_IM3'))
            transitions.append((None, 'C_IM3', z3p.Neq(AckCounter, NumAcksTemp), [(AckCounter, AckCounter - NumAcksTemp)], 'C_SM'))
            transitions.append((None, 'C_IM3', z3p.Eq(AckCounter, NumAcksTemp), [(AckCounter, z3p.IntVal(0))], 'C_IM4'))
            transitions.append((None, 'C_IM4', UnblockEMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [(DataBlk, PendingWrite), (PendingWrite, z3p.IntVal(0))], 'C_M'))

            # foreach c2 in CacheIDType (!= c2 c) {
            #     C_IM on InvAckMsg[c2][c][d][a] { AckCounter := (+ AckCounter 1); } -> C_IM;
            # }
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IM', InvAckMsgP[d][a][other_c], z3p.BoolVal(True), [(AckCounter, AckCounter + 1)], 'C_IM'))

            # // Transitions from C_SM on Ext Events
            # C_SM on LDMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_SM;
            transitions.append((None, 'C_SM', LDMsg[d][a], z3p.BoolVal(True), [], 'C_SM2'))
            transitions.append((None, 'C_SM2', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_SM'))

            # // Transitions from C_SM on Rsp Events
            # C_SM on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {} -> C_SM;
            transitions.append((None, 'C_SM', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_SM3'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_SM3', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [], 'C_SM'))


            # // Case where data comes from dir ==> Must wait for acks
            # C_SM on DataMsgD2C'[c][d][a] (v, NumAcks)

            transitions.append((None, 'C_SM', DataMsgD2CP[d][a], z3p.BoolVal(True), [(NumAcksTemp, DataMsgD2CP[d][a][1])], 'C_SM4'))

            # if (= NumAcks AckCounter) { AckCounter := 0; } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            # if (!= NumAcks AckCounter) { AckCounter := (- AckCounter NumAcks) } -> C_SM;
            transitions.append((None, 'C_SM4', UnblockEMsg[d][a], [z3p.IntVal(0)], z3p.Eq(NumAcksTemp, AckCounter), [(DataBlk, PendingWrite), (PendingWrite, z3p.IntVal(0))], 'C_SM5'))
            transitions.append((None, 'C_SM5', STAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))
            transitions.append((None, 'C_SM4', z3p.Neq(NumAcksTemp, AckCounter), [(AckCounter, AckCounter - NumAcksTemp)], 'C_SM'))

            # foreach c2 in CacheIDType (!= c2 c) {
            #     C_SM on InvAckMsg'[c2][c][d][a]
            # if (= AckCounter (- 1)) { AckCounter := 0; } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            # if (!= AckCounter (- 1)) { AckCounter := (+ AckCounter 1); } ->
            # send UnblockEMsg[c][d][a] { DataBlk := PendingStore; PendingStore := undef; } ->
            # send STAckMsg[c][d][a] (DataBlk) {} -> C_M;
            # }
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_SM', InvAckMsgP[d][a][other_c], z3p.BoolVal(True), [], 'C_SM5'))
                    transitions.append((None, 'C_SM5', UnblockEMsg[d][a], [z3p.IntVal(0)], z3p.Eq(AckCounter, z3p.IntVal(0) - z3p.IntVal(1)), [(AckCounter, z3p.IntVal(0)), (DataBlk, PendingWrite), (PendingWrite, z3p.IntVal(0))], 'C_SM6'))
                    transitions.append((None, 'C_SM6', STAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))

                    transitions.append((None, 'C_SM5', UnblockEMsg[d][a], [z3p.IntVal(0)], z3p.Neq(AckCounter, z3p.IntVal(0) - z3p.IntVal(1)), [(AckCounter, AckCounter + 1), (DataBlk, PendingWrite), (PendingWrite, z3p.IntVal(0))], 'C_SM7'))
                    transitions.append((None, 'C_SM7', STAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_M'))

            # // Transitions from IS on Ext Events
            # C_IS on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] {} -> C_IS;
            transitions.append((None, 'C_IS', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_IS2'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IS2', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(z3p.IntVal(other_c), FwdToCache), [], 'C_IS'))

            # // Transitions on IS on Rsp Events
            # C_IS on DataMsgD2C'[c][d][a] (v, _) { DataBlk := v; } ->
            # send UnblockSMsg[c][d][a] {} -> send LDAckMsg[c][d][a] (DataBlk) {} -> C_S;
            transitions.append((None, 'C_IS', DataMsgD2CP[d][a], z3p.BoolVal(True), [(DataBlk, DataMsgD2CP[d][a][0])], 'C_IS2'))
            transitions.append((None, 'C_IS2', UnblockSMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_IS3'))
            transitions.append((None, 'C_IS3', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_S'))

            # foreach c2 in CacheIDType (!= c2 c) {
            # C_IS on DataMsgC2C'[c2][c][d][a] (v) -> { DataBlk := v; } ->
            # send UnblockSMsg[c][d][a] -> send LDAckMsg[c][d][a] (DataBlk) -> C_S;
            # }
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_IS', DataMsgC2CP[d][a][other_c], z3p.BoolVal(True), [(DataBlk, DataMsgC2CP[d][a][other_c])], 'C_IS4'))
                    transitions.append((None, 'C_IS4', UnblockSMsg[d][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'C_IS5'))
                    transitions.append((None, 'C_IS5', LDAckMsg[d][a], [DataBlk], z3p.BoolVal(True), [], 'C_S'))

            # // Transitions on II on Rsp Events
            # C_II on WBAckMsg'[c][d][a] -> C_I;
            transitions.append((None, 'C_II', WBAckMsgP[d][a], z3p.BoolVal(True), [], 'C_I'))

            # C_II on FwdGetXMsg'[c][d][a] (c2) { FwdToCache := c2; } ->
            # send InvAckMsg[c][FwdToCache][d][a] -> C_I;
            transitions.append((None, 'C_II', FwdGetXMsgP[d][a], z3p.BoolVal(True), [(FwdToCache, FwdGetXMsgP[d][a])], 'C_II2'))
            for other_c in range(num_caches):
                if other_c != cache_id:
                    transitions.append((None, 'C_II2', InvAckMsg[d][a][other_c], [z3p.IntVal(0)], z3p.Eq(FwdToCache, other_c), [], 'C_I'))

            # C_II on FwdGetSMsg'[c][d][a] (_) -> I;
            # }
            transitions.append((None, 'C_II', FwdGetSMsgP[d][a], z3p.BoolVal(True), [], 'C_I'))


    locations = ['C_I', 'C_S', 'C_M', 'C_IM', 'C_SM', 'C_IS', 'C_II']
    locations.extend(['C_I2', 'C_I3', 'C_I4', 'C_I5'])
    locations.extend(['C_S2', 'C_S3', 'C_S4', 'C_S5'])
    locations.extend(['C_M2', 'C_M3', 'C_M4', 'C_M5', 'C_M6'])
    locations.extend(['C_IM2', 'C_IM3', 'C_IM4'])
    locations.extend(['C_SM2', 'C_SM3', 'C_SM4', 'C_SM5', 'C_SM6', 'C_SM7'])
    locations.extend(['C_IS2', 'C_IS3', 'C_IS4', 'C_IS5'])
    locations.extend(['C_II2'])

    return automaton.SymbolicAutomaton('cache{}'.format(cache_id),
                                       locations,
                                       'C_I',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=input_channels,
                                       output_channels=output_channels,
                                       variable_ranges=variable_ranges)


def directory(directory_id, num_caches=2, num_values=2, num_addresses=2):
    transitions = []
    variables = []
    variable_ranges = {}
    DataBlk = z3p.Int('DataBlk')
    ActiveId = z3p.Int('ActiveId')
    Sharers = {}
    for cache_id in range(num_caches):
        Sharers[cache_id] = z3p.Int('Sharers_{}'.format(cache_id))
    NumSharers = z3p.Int('NumSharers')
    Owner = z3p.Int('Owner')

    variable_ranges = {DataBlk: (0, num_values - 1),
                       ActiveId: (0, num_caches - 1),
                       NumSharers: (0, num_caches),
                       Owner: (0, num_caches - 1)}
    for cache_id in range(num_caches):
        variable_ranges[Sharers[cache_id]] = (0, 1)


    variables = [DataBlk, ActiveId, NumSharers, Owner] + [Sharers[cache_id] for cache_id in range(num_caches)]

    initial_values = [z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0), z3p.IntVal(0)] + num_caches * [z3p.IntVal(0)]

    locations = ['D_I', 'D_S', 'D_M', 'D_BUSY', 'D_DATA', 'D_PENDING_UNBLOCK_E', 'D_BUSY_DATA']

    input_channels = []
    # input messages
    GetXMsgP = {}
    GetSMsgP = {}
    WBMsgP = {}
    UnblockSMsgP = {}
    UnblockEMsgP = {}
    DataMsgC2CP = {}
    for c in range(num_caches):
        GetXMsgP[c] = {}
        GetSMsgP[c] = {}
        WBMsgP[c] = {}
        UnblockSMsgP[c] = {}
        UnblockEMsgP[c] = {}
        DataMsgC2CP[c] = {}
        for a in range(num_addresses):
            suffix = '_{}_{}_{}'.format(c, directory_id, a)
            GetXMsgP[c][a] = util.new_variable('GetXMsgP' + suffix, 'unit')
            input_channels.append(GetXMsgP[c][a])
            GetSMsgP[c][a] = util.new_variable('GetSMsgP' + suffix, 'unit')
            input_channels.append(GetSMsgP[c][a])
            WBMsgP[c][a] = z3p.Int('WBMsgP' + suffix)
            input_channels.append(WBMsgP[c][a])
            UnblockSMsgP[c][a] = util.new_variable('UnblockSMsgP' + suffix, 'unit')
            input_channels.append(UnblockSMsgP[c][a])
            UnblockEMsgP[c][a] = util.new_variable('UnblockEMsgP' + suffix, 'unit')
            input_channels.append(UnblockEMsgP[c][a])
            DataMsgC2CP[c][a] = {}
            for other_c in range(num_caches):
                if other_c != c:
                    suffix2 = '_{}{}'.format(other_c, suffix)
                    DataMsgC2CP[c][a][other_c] = z3p.Int('DataMsgC2CP' + suffix2)
                    input_channels.append(DataMsgC2CP[c][a][other_c])

    output_channels = []
    # output messages
    FwdGetXMsg = {}
    FwdGetSMsg = {}
    DataMsgD2C = {}
    WBAckMsg = {}
    for c in range(num_caches):
        FwdGetXMsg[c] = {}
        FwdGetSMsg[c] = {}
        DataMsgD2C[c] = {}
        WBAckMsg[c] = {}
        for a in range(num_addresses):
            suffix = '_{}_{}_{}'.format(c, directory_id, a)
            FwdGetXMsg[c][a] = z3p.Int('FwdGetXMsg' + suffix)
            output_channels.append(FwdGetXMsg[c][a])
            FwdGetSMsg[c][a] = z3p.Int('FwdGetSMsg' + suffix)
            output_channels.append(FwdGetSMsg[c][a])
            DataMsgD2C[c][a] = z3p.Array('DataMsgD2C' + suffix, z3p.IntSort(), z3p.IntSort())
            output_channels.append(DataMsgD2C[c][a])
            WBAckMsg[c][a] = util.new_variable('WBAckMsg' + suffix, 'unit')
            output_channels.append(WBAckMsg[c][a])


    for c in range(num_caches):
        for a in range(num_addresses):
            # // Transitions from I
            #  D_I on GetXMsg'[c][d][a] { ActiveId := c; } ->
            #  send DataMsgD2C[c][d][a] (DataBlk, 0) -> D_BUSY;
            transitions.append((None, 'D_I', GetXMsgP[c][a], z3p.BoolVal(True), [(ActiveId, z3p.IntVal(c))], 'D_I2'))
            transitions.append((None, 'D_I2', DataMsgD2C[c][a], [DataBlk, z3p.IntVal(0)], z3p.BoolVal(True), [], 'D_BUSY'))
            #  D_I on GetSMsg'[c][d][a] { ActiveId := c; } ->
            #  send DataMsgD2C[c][d][a] (DataBlk, 0) -> D_BUSY;
            transitions.append((None, 'D_I', GetSMsgP[c][a], z3p.BoolVal(True), [(ActiveId, z3p.IntVal(c))], 'D_I3'))
            transitions.append((None, 'D_I3', DataMsgD2C[c][a], [DataBlk, z3p.IntVal(0)], z3p.BoolVal(True), [], 'D_BUSY'))
            #  // Transitions from S
            #  D_S on GetXMsg'[c][d][a] { ActiveId := c; } ->
            #  send DataMsgD2C[c][d][a] (DataBlk, NumSharers) {} ->
            #  foreach c2 in CacheIDType {
            #      if (Sharers[c2]) send FwdGetXMsg[c2][d][a] (c) {};
            #      if (not Sharers[c2]) pass {};
            #  } -> D_BUSY;
            transitions.append((None, 'D_S', GetXMsgP[c][a], z3p.BoolVal(True), [(ActiveId, z3p.IntVal(c))], 'D_S2'))
            transitions.append((None, 'D_S2', DataMsgD2C[c][a], [DataBlk, NumSharers], z3p.BoolVal(True), [], 'D_S3'))

            for i, other_c in enumerate(range(num_caches)):
                if i == 0:
                    source = 'D_S3'
                else:
                    source = 'D_S3_{}'.format(i)
                if i + 1 == num_caches:
                    target = 'D_BUSY'
                else:
                    target = 'D_S3_{}'.format(i + 1)
                transitions.append((None, source, FwdGetXMsg[other_c][a], [z3p.IntVal(c)], z3p.Eq(Sharers[other_c], z3p.IntVal(1)), [], target))
                locations.extend(['D_S3_{}'.format(i) for i in range(1, num_caches)])

            #  D_S on GetSMsg'[c][d][a] { ActiveId := c; } ->
            #  send DataMsgD2C[c][d][a] (DataBlk, 0) {} -> D_BUSY;
            transitions.append((None, 'D_S', GetSMsgP[c][a], z3p.BoolVal(True), [(ActiveId, z3p.IntVal(c))], 'D_S4'))
            transitions.append((None, 'D_S4', DataMsgD2C[c][a], [DataBlk, z3p.IntVal(0)], z3p.BoolVal(True), [], 'D_BUSY'))
            #  // Transitions from M
            #  D_M on GetXMsg'[c][d][a] { ActiveId := c; } ->
            #  send FwdGetXMsg[Owner][d][a] (ActiveId) {} -> D_BUSY;
            transitions.append((None, 'D_M', GetXMsgP[c][a], z3p.BoolVal(True), [(ActiveId, z3p.IntVal(c))], 'D_M2'))
            for c2 in range(num_caches):
                transitions.append((None, 'D_M2', FwdGetXMsg[c2][a], [ActiveId], z3p.Eq(z3p.IntVal(c2), Owner), [], 'D_BUSY'))
            #  D_M on GetSMsg'[c][d][a] { ActiveId := c; } ->
            #  send FwdGetSMsg[Owner][d][a] (ActiveId) {} -> D_BUSY_DATA;
            transitions.append((None, 'D_M', GetSMsgP[c][a], z3p.BoolVal(True), [(ActiveId, z3p.IntVal(c))], 'D_M3'))
            for c2 in range(num_caches):
                transitions.append((None, 'D_M3', FwdGetSMsg[c2][a], [ActiveId], z3p.Eq(z3p.IntVal(c2), Owner), [], 'D_BUSY_DATA'))
            #  D_M on WBMsg'[c][d][a] (v) { DataBlk := v; Sharers[c] := false; } ->
            #  send WBAckMsg[c][d][a] {} -> D_I;
            transitions.append((None, 'D_M', WBMsgP[c][a], z3p.BoolVal(True), [(DataBlk, WBMsgP[c][a]), (Sharers[c], z3p.IntVal(1))], 'D_M4'))
            transitions.append((None, 'D_M4', WBAckMsg[c][a], [z3p.IntVal(0)], z3p.BoolVal(True), [], 'D_I'))
            #  // Transitions from BUSY
            #  D_BUSY on WBMsg'[c][d][a] (v)
            #  if (= c ActiveId) { DataBlk := v; } -> D_PENDING_UNBLOCK_E;
            #  if (!= c ActiveId) { Sharers[c] := false; DataBlk := v; } ->
            #  send DataMsgD2C[ActiveId][d][a] (DataBlk, 0) -> D_BUSY;
            transitions.append((None, 'D_BUSY', WBMsgP[c][a], z3p.Eq(z3p.IntVal(c), ActiveId), [(DataBlk, WBMsgP[c][a])], 'D_PENDING_UNBLOCK_E'))
            transitions.append((None, 'D_BUSY', WBMsgP[c][a], z3p.Neq(z3p.IntVal(c), ActiveId), [(Sharers[c], z3p.IntVal(0)), (DataBlk, WBMsgP[c][a])], 'D_BUSY2'))
            for c2 in range(num_caches):
                transitions.append((None, 'D_BUSY2', DataMsgD2C[c2][a], [DataBlk, z3p.IntVal(0)], z3p.Eq(z3p.IntVal(c2), ActiveId), [], 'D_BUSY'))
            #  D_BUSY on UnblockEMsg'[c][d][a] { Sharers[c] := true; Owner := c; } -> D_M;
            transitions.append((None, 'D_BUSY', UnblockEMsgP[c][a], z3p.BoolVal(True), [(Sharers[c], z3p.IntVal(1)), (Owner, z3p.IntVal(c))], 'D_M'))
            #  D_BUSY on UnblockSMsg'[c][d][a] { Sharers[c] := true; Owner := undef; } -> D_S;
            transitions.append((None, 'D_BUSY', UnblockSMsgP[c][a], z3p.BoolVal(True), [(Sharers[c], z3p.IntVal(1)), (Owner, z3p.IntVal(0))], 'D_S'))
            #  foreach c2 in CacheIDType (!= c2 c) {
            #      D_BUSY on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> S;
            #  }
            for c2 in range(num_caches):
                if c2 != c:
                    transitions.append((None, 'D_BUSY', DataMsgC2CP[c][a][c2], z3p.BoolVal(True), [(DataBlk, DataMsgC2CP[c][a][c2])], 'D_S'))
            #  // Transitions from BUSY_DATA
            #  D_BUSY_DATA on UnblockSMsg'[c][d][a] { Sharers[c] := true; } -> D_BUSY;
            #  foreach c2 in CacheIDType (!= c2 c) {
            #      D_BUSY_DATA on DataMsgC2C'[c2][c][d][a] (v) { DataBlk := v; } -> D_BUSY;
            #  }
            transitions.append((None, 'D_BUSY_DATA', UnblockSMsgP[c][a], z3p.BoolVal(True), [(Sharers[c], z3p.IntVal(1))], 'D_BUSY'))
            for c2 in range(num_caches):
                if c2 != c:
                    transitions.append((None, 'D_BUSY_DATA', DataMsgC2CP[c][a][c2], z3p.BoolVal(True), [(DataBlk, DataMsgC2CP[c][a][c2])], 'D_BUSY'))
            #  D_BUSY_DATA on WBMsg'[c][d][a] (v)
            #  if (= c ActiveId) { DataBlk := v; } -> D_PENDING_UNBLOCK_E;
            #  if (!= c ActiveId) { Sharers[c] := false; DataBlk := v; } ->
            #  send DataMsgD2C[ActiveId][d][a] (DataBlk, 0) -> D_BUSY;
            transitions.append((None, 'D_BUSY_DATA', WBMsgP[c][a], z3p.Eq(z3p.IntVal(c), ActiveId), [(DataBlk, WBMsgP[c][a])], 'D_PENDING_UNBLOCK_E'))
            transitions.append((None, 'D_BUSY_DATA', WBMsgP[c][a], z3p.Neq(z3p.IntVal(c), ActiveId), [(Sharers[c], z3p.IntVal(0)), (DataBlk, WBMsgP[c][a])], 'D_BUSY_DATA2'))
            for c2 in range(num_caches):
                transitions.append((None, 'D_BUSY_DATA2', DataMsgD2C[c2][a], [DataBlk, z3p.IntVal(0)], z3p.Eq(z3p.IntVal(c2), ActiveId), [], 'D_BUSY'))
            #  // Transitions from PENDING_UNBLOCK_E
            #  D_PENDING_UNBLOCK_E on UnblockEMsg'[c][d][a] { Sharers[c] := false; Owner := undef; } -> D_I;
            transitions.append((None, 'D_PENDING_UNBLOCK_E', UnblockEMsgP[c][a], z3p.BoolVal(True), [(Sharers[c], z3p.IntVal(0)), (Owner, z3p.IntVal(0))], 'D_I'))

    locations.extend(['D_I2', 'D_I3'])
    locations.extend(['D_S2', 'D_S3', 'D_S4'])
    locations.extend(['D_M2', 'D_M3', 'D_M4'])
    locations.extend(['D_BUSY2'])
    locations.extend(['D_BUSY_DATA2'])

    return automaton.SymbolicAutomaton('directory',
                                       locations,
                                       'D_I',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=input_channels,
                                       output_channels=output_channels,
                                       variable_ranges=variable_ranges)

def request_channel(address_id, cache_id, directory_id, num_values):
    name = 'request_channel_{}_{}_{}'.format(address_id, cache_id, directory_id)
    inputs = []
    outputs = []
    suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
    for port in ['GetXMsg', 'GetSMsg']:
        inputs.append(util.new_variable(port + suffix, 'unit'))
        outputs.append(util.new_variable(port + 'P' + suffix, 'unit'))
    inputs.append(z3p.Int('WBMsg' + suffix))
    outputs.append(z3p.Int('WBMsgP' + suffix))
    data_ranges = {z3p.Int('WBMsg' + suffix): (0, num_values - 1)}
    return lossless_non_duplicating_blocking_capacity_one_channel(name, inputs, outputs, data_ranges, {})


def response_channel(address_id, cache_id, directory_id, num_values, num_caches):
    name = 'response_channel_{}_{}_{}'.format(address_id, cache_id, directory_id)
    inputs = []
    outputs = []
    data_ranges = {}
    number_of_fields = {}
    suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
    wbackmsg = 'WBAckMsg'
    wbackmsg_input = util.new_variable(wbackmsg + suffix, 'unit')
    wbackmsg_output = util.new_variable(wbackmsg + 'P' + suffix, 'unit')
    inputs.append(wbackmsg_input)
    outputs.append(wbackmsg_output)
    # data_ranges[wbackmsg_input] = (0, 0)
    # number_of_fields[wbackmsg_input] = 1

    datamsgd2c = 'DataMsgD2C'
    datamsgd2c_input = z3p.Array(datamsgd2c + suffix, z3p.IntSort(), z3p.IntSort())
    datamsgd2c_output = z3p.Array(datamsgd2c + 'P' + suffix, z3p.IntSort(), z3p.IntSort())
    inputs.append(datamsgd2c_input)
    outputs.append(datamsgd2c_output)
    data_ranges[datamsgd2c_input[0]] = (0, num_values - 1)
    data_ranges[datamsgd2c_input[1]] = (0, num_caches)
    number_of_fields[datamsgd2c_input] = 2

    datamsgc2c = 'DataMsgC2C'
    invackmsg = 'InvAckMsg'
    for other_cache_id in range(num_caches):
        if other_cache_id != cache_id:
            suffix2 = '_{}{}'.format(other_cache_id, suffix)
            datamsgc2c_input = z3p.Int(datamsgc2c + suffix2)
            datamsgc2c_output = z3p.Int(datamsgc2c + 'P' + suffix2)
            print datamsgc2c_input
            inputs.append(datamsgc2c_input)
            outputs.append(datamsgc2c_output)
            data_ranges[datamsgc2c_input] = (0, num_values - 1)
            invackmsg_input = util.new_variable(invackmsg + suffix2, 'unit')
            invackmsg_input = z3p.Int(invackmsg + suffix2)
            invackmsg_output = util.new_variable(invackmsg + 'P' + suffix2, 'unit')
            inputs.append(invackmsg_input)
            outputs.append(invackmsg_output)
            # data_ranges[invackmsg_input] = (0, 0)
            # number_of_fields[datamsgc2c_input] = 1
    return lossless_non_duplicating_blocking_capacity_one_channel(name, inputs, outputs, data_ranges, number_of_fields)
    # return lossless_non_duplicating_blocking_unordered_channel(name, inputs, outputs, data_ranges, number_of_fields, capacity=1)


def unblock_channel(address_id, directory_id, num_caches):
    name = 'unblock_channel_{}_{}'.format(address_id, directory_id)
    inputs = []
    outputs = []
    for cache_id in range(num_caches):
        suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
        for port in ['UnblockEMsg', 'UnblockSMsg']:
            inputs.append(util.new_variable(port + suffix, 'unit'))
            outputs.append(util.new_variable(port + 'P' + suffix, 'unit'))
    return lossless_non_duplicating_blocking_capacity_one_channel(name, inputs, outputs, {}, {})


def forward_channel(cache_id, directory_id, address_id, num_caches):
    name = 'forward_channel_{}_{}_{}'.format(cache_id, directory_id, address_id)
    inputs = []
    outputs = []
    suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
    data_ranges = {}
    for port in ['FwdGetSMsg', 'FwdGetXMsg']:
        inputs.append(z3p.Int(port + suffix))
        outputs.append(z3p.Int(port + 'P' + suffix))
        data_ranges[z3p.Int(port + suffix)] = (0, num_caches - 1)
    return lossless_non_duplicating_blocking_capacity_one_channel(name, inputs, outputs, data_ranges, {})


def environment(cache_id, directory_id, address_id, num_values):
    transitions = []
    variables = []
    variable_ranges = {}
    PendingStore = z3p.Int('pending_store_environment_{}_{}_{}'.format(cache_id, directory_id, address_id))
    StoreResult = z3p.Int('store_result_{}_{}_{}'.format(cache_id, directory_id, address_id))

    variable_ranges[PendingStore] = (0, num_values - 1)
    variable_ranges[StoreResult] = (0, num_values - 1)
    variables = [PendingStore, StoreResult]

    initial_values = [z3p.IntVal(0), z3p.IntVal(0)]

    locations = ['Env_Initial', 'Env_PendingLD', 'Env_PendingST', 'Env_PendingEV', 'Env_Error']

    # input messages
    suffix = '_{}_{}_{}'.format(cache_id, directory_id, address_id)
    LDAckMsg = z3p.Int('LDAckMsg' + suffix)
    STAckMsg = z3p.Int('STAckMsg' + suffix)
    EVAckMsg = util.new_variable('EVAckMsg' + suffix, 'unit')
    input_channels = [LDAckMsg, STAckMsg, EVAckMsg]

    LDMsg = util.new_variable('LDMsg' + suffix, 'unit')
    STMsg = z3p.Int('STMsg' + suffix)
    EVMsg = util.new_variable('EVMsg' + suffix, 'unit')
    output_channels = [LDMsg, STMsg, EVMsg]

    transitions = []
    # // Load flow
    # Env_Initial send LDMsg[c][d][a] {} -> Env_PendingLD;
    # Env_PendingLD on LDAckMsg[c][d][a] (v) {} -> Env_Initial;
    transitions.append((None, 'Env_Initial', LDMsg, [z3p.IntVal(0)], z3p.BoolVal(True), [], 'Env_PendingLD'))
    transitions.append((None, 'Env_PendingLD', LDAckMsg, z3p.BoolVal(True), [], 'Env_Initial'))

    # // Store flow
    # foreach v in ValueType
    # Env_Initial send STMsg[c][d][a] (v) { PendingStore := v; } -> Env_PendingST;
    # Env_PendingST on STAckMsg[c][d][a]
    # if (v = PendingStore) {} -> Env_Initial;
    # if (v != PendingStore) {} -> Env_Error;
    for v in range(num_values):
        transitions.append((None, 'Env_Initial', STMsg, [z3p.IntVal(v)], z3p.BoolVal(True), [(PendingStore, z3p.IntVal(v))], 'Env_PendingST'))
        transitions.append((None, 'Env_PendingST', STAckMsg, z3p.BoolVal(True), [(StoreResult, STAckMsg)], 'Env_PendingST2'))
        transitions.append((None, 'Env_PendingST2', z3p.Eq(StoreResult, PendingStore), [(StoreResult, z3p.IntVal(0)), (PendingStore, z3p.IntVal(0))], 'Env_Initial'))
        transitions.append((None, 'Env_PendingST2', z3p.Neq(StoreResult, PendingStore), [(StoreResult, z3p.IntVal(0)), (PendingStore, z3p.IntVal(0))], 'Env_Error'))

    # // Evict flow
    # Env_Initial send EVMsg[c][d][a] {} -> Env_PendingEV;
    # Env_PendingEV on EVAckMsg[c][d][a] {} -> Env_Initial;
    transitions.append((None, 'Env_Initial', EVMsg, [z3p.IntVal(0)], z3p.BoolVal(True), [], 'Env_PendingEV'))
    transitions.append((None, 'Env_PendingEV', EVAckMsg, z3p.BoolVal(True), [], 'Env_Initial'))

    locations.extend(['Env_PendingST2'])

    return automaton.SymbolicAutomaton('enviroment_{}_{}_{}'.format(cache_id, directory_id, address_id),
                                       locations,
                                       'Env_Initial',
                                       transitions,
                                       variables=variables,
                                       initial_values=initial_values,
                                       input_channels=input_channels,
                                       output_channels=output_channels,
                                       variable_ranges=variable_ranges)

NUM_ADDRESSES = 1
NUM_CACHES = 2
NUM_DIRECTORIES = 1
NUM_VALUES = 2

automata = ([forward_channel(cache_id, directory_id, address_id, NUM_CACHES) for address_id in range(NUM_ADDRESSES) for cache_id in range(NUM_CACHES) for directory_id in range(NUM_DIRECTORIES)]+
            [directory(directory_id, NUM_CACHES, NUM_VALUES, NUM_ADDRESSES) for directory_id in range(NUM_DIRECTORIES)] +
            [cache(cache_id, NUM_CACHES, NUM_VALUES, NUM_ADDRESSES, NUM_DIRECTORIES) for cache_id in range(NUM_CACHES)] +
            [request_channel(address_id, cache_id, directory_id, NUM_VALUES) for address_id in range(NUM_ADDRESSES) for cache_id in range(NUM_CACHES) for directory_id in range(NUM_DIRECTORIES)] +
            [response_channel(address_id, cache_id, directory_id, NUM_VALUES, NUM_CACHES) for address_id in range(NUM_ADDRESSES) for cache_id in range(NUM_CACHES) for directory_id in range(NUM_DIRECTORIES)] +
            [unblock_channel(address_id, directory_id, NUM_CACHES) for address_id in range(NUM_ADDRESSES) for directory_id in range(NUM_DIRECTORIES)] +
            [environment(cache_id, directory_id, address_id, NUM_VALUES) for address_id in range(NUM_ADDRESSES) for cache_id in range(NUM_CACHES) for directory_id in range(NUM_DIRECTORIES)]
)
